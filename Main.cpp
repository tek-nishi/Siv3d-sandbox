
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


struct Player
{
  Vec3 pos;            // 位置
  float direction;     // 向き

  Vec3 velocity;       // 速度
  float acceleration;  // 加速度

  Vec2 size;
};

struct Shot
{
  Vec3 pos;
  Vec3 velocity;
  float size;
  bool active;
};


struct Rock
{
  Vec3 pos;
  Vec3 velocity;
  float size;
  float rotate;
  int hp;
};


void Main()
{
  Window::Resize(800, 800);

  Player player{
    { 0, 0, 0 },
    0,

    { 0, 0, 0 },
    -0.5,

    { 60, 50 }
  };

  Array<Shot> shots;
  int shot_count = 0;

  Array<Rock> rocks;
  for (int i = 0; i < 10; ++i)
  {
    auto pos = Vec3{ Random(-500, 500), Random(-500, 500), 0 };
    auto v   = Vec3{ 0, 0, 0 };
    float s  = Random(40.0f, 80.0f);
    float r  = Random(-90, 90);
    rocks << Rock{ pos, v, s, r, 2 };
  }


  Array<Vec3> stars;
  for (int i = 0; i < 100; ++i)
  {
    stars << Vec3{ Random(-500, 500), Random(-500, 500), 0 }; 
  }

  Vec3 camera = player.pos;


  while (System::Update())
  {
    {
      // 向きを決める
      Vec2 d = Cursor::Pos() - Window::Center();
      d.normalize();
      player.direction = std::atan2(d.y, d.x);

      // 移動
      Vec3 acc{ 0, 0, 0 };
      if (MouseL.pressed())
      {
        if (!shot_count)
        {
          // 弾発射
          shots << Shot{ player.pos, player.velocity + Vec3(d.x, d.y, 0) * 5, 6, true };

          acc = Vec3{ d.x, d.y, 0 } * player.acceleration;
          shot_count = 8;
          // Print << shots.size();
        }
        else
        {
          shot_count -= 1;
        }
      }
      else if (MouseL.up())
      {
          shot_count = 0;
      }
      acc += player.velocity * -0.01;
      player.pos += player.velocity + acc * 0.5;
      player.velocity += acc;
    }

    // 弾移動
    for (auto& s : shots)
    {
      s.pos += s.velocity;
    }

    // 岩移動
    for (auto& obj : rocks)
    {
      obj.pos += obj.velocity;
    }

    // 弾と岩の重なり判定
    for (auto& rock : rocks)
    {
      auto rock_obj = Polygon(Shape2D::Hexagon(rock.size, Vec2{ rock.pos.x, rock.pos.y }, rock.rotate));

      for (auto& shot : shots)
      {
        auto shot_obj = Polygon(Shape2D::Star(10, Vec2{ shot.pos.x, shot.pos.y }));

        if (rock_obj.intersects(shot_obj))
        {
          rock.hp -= 1;
          rock.velocity += shot.velocity * 0.1;

          shot.active = false;

          break;
        }
      }
    }

    // 画面外判定とか
    shots.remove_if([&camera](const Shot& shot)
                    {
                      auto rect = Window::ClientRect();
                      auto lp = shot.pos - camera;
                      auto sp = Vec2{ lp.x, lp.y } + Window::Center(); 

                      return !shot.active
                             || sp.x < 0
                             || sp.x > rect.w 
                             || sp.y < 0
                             || sp.y > rect.h;
                    });
    
    // 
    rocks.remove_if([](const Rock& obj)
                   {
                     return obj.hp < 0;
                    });


    // カメラ
    camera += (player.pos - camera) * 0.05;


    // 星を表示
    for (const auto& p : stars)
    {
      auto lp = p - camera;
      auto pos = Vec2{ lp.x, lp.y } + Window::Center();
      Shape2D::Star(10, pos).draw(Palette::Yellow);
    }

    for (const auto& obj : rocks)
    {
      auto lp = obj.pos - camera;
      auto pos = Vec2{ lp.x, lp.y } + Window::Center();
      Shape2D::Hexagon(obj.size, pos, obj.rotate).draw(Palette::Orange);
    }

    // 弾を表示
    for (const auto& s : shots)
    {
      auto lp = s.pos - camera;
      auto pos = Vec2{ lp.x, lp.y } + Window::Center();
      Shape2D::Pentagon(s.size, pos).draw(Palette::Red);
    }

    // プレイヤー表示
    {
      auto lp = player.pos - camera;

      auto p = Vec2{ lp.x, lp.y } + Window::Center() - player.size * 0.5;
      Rect(p.x, p.y, player.size.x, player.size.y).rotated(player.direction).draw();
    }
  }
}

