
# include <Siv3D.hpp>


Vec3 rotatePitch(Vec3 pos, float pitch)
{
  auto sin_t = std::sin(pitch);
  auto cos_t = std::cos(pitch);

  return { pos.x,
           pos.y * cos_t + pos.z * -sin_t,
           pos.y * sin_t + pos.z * cos_t };
}

Vec3 rotateYaw(Vec3 pos, float yaw)
{
  auto sin_t = std::sin(yaw);
  auto cos_t = std::cos(yaw);

  return { pos.x * cos_t + pos.z * sin_t,
           pos.y,
           pos.x * -sin_t + pos.z * cos_t };
}


//軸ベクトルと回転量から生成
Vec4 createQuaternion(const Vec3& v, float r)
{
  float half_sin = std::sin(r / 2.0f);
  return { v.x * half_sin,
           v.y * half_sin,
           v.z * half_sin,
           std::cos(r / 2.0) };
}

// 積
Vec4 mulQuaternion(const Vec4& q1, const Vec4& q2)
{
  Vec3 v1{ q1.x, q1.y, q1.z };
  Vec3 v2{ q2.x, q2.y, q2.z };

  auto v = v1.cross(v2) + v1 * q2.w;
  v += v2 * q1.w;

  return { v.x, v.y, v.z, q1.w * q2.w - v1.dot(v2) };
}

// 共役
Vec4 conjugateQuaternion(const Vec4& q)
{
  return { -q.x, -q.y, -q.z, q.w };
}

// 点pをクォータニオンで変換
Vec3 transformVector(Vec3 pos, const Vec4& q)
{
  auto qc = conjugateQuaternion(q);

  Vec4 v{ pos.x, pos.y, pos.z, 0 };
  
  auto pq  = mulQuaternion(q, v);
  auto pqp = mulQuaternion(pq, qc);

  return { pqp.x, pqp.y, pqp.z };
}


void Main()
{
  Window::Resize(800, 600);

  Array<Vec3> points;

  // 点を用意
  for (int x = -5; x <= 5; ++x)
  {
    for (int y = -5; y <= 5; ++y)
    {
      points << Vec3(x * 20, y * 20, 0);
    }
  }

  float r = 0;
  Vec4 rotation{ 0, 0, 0, 1 }; 

  while (System::Update())
  {
    r += 0.02;

    // マウスドラッグで回転
    if (MouseL.pressed())
    {
      auto v1 = Vec3(Cursor::PreviousPos() - Window::Center(), -100).normalized();
      auto v2 = Vec3(Cursor::Pos() - Window::Center(), -100).normalized();

      if (v1 != v2)
      {
        Vec3 v3 = v1.cross(v2);
        float r = std::asin(v3.length());
        auto q  = createQuaternion(v3.normalized(), r);
        rotation = mulQuaternion(q, rotation);
      }
    }


    struct P {
      Vec3 pos;
      float r;
    };

    Array<P> draw_points;

    // 全ての座標を透視変換する
    int num = points.size();
    int i = 0;
    for (const auto& p : points)
    {
      // アフィン変換
      // auto p3 = rotatePitch(p, r);
      // p3 = rotateYaw(p3, r);

      // auto q = createQuaternion(Vec3(1, 1, 0.5).normalize(), r);
      auto p3 = transformVector(p, rotation);

      // 適当透視変換
      float zd = ((p3.z + 300) * 0.002);
      float x = p3.x / zd; 
      float y = p3.y / zd;
      float z = 8 / zd;

      float r = 360.0 * i / num;
      draw_points.push_back({ { x, y, z }, r });

      ++i;
    }

    // Z値でソート
    draw_points.sort_by([](P& a, P& b)
                        {
                          return a.pos.z < b.pos.z;
                        });

    // 表示
    for (const auto& p : draw_points)
    {
      Vec2 p2{ p.pos.x, p.pos.y };
      Circle(p2 + Window::Center(), p.pos.z).draw(HSV(p.r));
    }
  }
}

