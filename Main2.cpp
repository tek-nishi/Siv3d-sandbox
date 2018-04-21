
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

  while (System::Update())
  {
    r += 0.02;

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
      auto p3 = rotatePitch(p, r);
      p3 = rotateYaw(p3, r);

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

