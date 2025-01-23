#pragma once

#include "libxr.hpp"
#include "main.hpp"
#include "thread.hpp"
#include <SDL.h>
#include <SDL_image.h>
#include <SDL_timer.h>
#include <SDL_ttf.h>
#include <cmath>
#include <unistd.h>

class NaoRobot;

#define HEIGHT 720
#define WIDTH 1024
#define PI 3.14159265358979323846

class Sim2D {
public:
  Sim2D() {
    sim2d = this;
    void (*thread_fun)(Sim2D *) = [](Sim2D *sim) {
      sim->Init();
      bool running = true;
      SDL_Event event;
      while (running) {
        while (SDL_PollEvent(&event)) {
          sim->Update();

          if (event.type == SDL_QUIT) { // 用户请求关闭窗口
            running = false;
          }
        }

        sim->Update();
      }
    };

    thread.Create(this, thread_fun, "sim2d", 4096,
                  LibXR::Thread::Priority::REALTIME);
  }

  void Init() {
    SDL_Init(SDL_INIT_EVERYTHING);
    IMG_Init(IMG_INIT_JPG | IMG_INIT_PNG);
    TTF_Init();

    window = SDL_CreateWindow("Example", SDL_WINDOWPOS_UNDEFINED,
                              SDL_WINDOWPOS_UNDEFINED, WIDTH, HEIGHT,
                              SDL_WINDOW_SHOWN);

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    font = TTF_OpenFont("../font.ttf", 24); // 加载字体文件
    if (!font) {
      printf("Failed to load font: %s\n", TTF_GetError());
      SDL_Quit();
      exit(-1);
    }

    auto img_field = IMG_Load("../field.jpg");
    if (!img_field) {
      printf("Failed to load image: %s\n", IMG_GetError());
      SDL_Quit();
      exit(-1);
    }

    texture = SDL_CreateTextureFromSurface(renderer, img_field);
    SDL_FreeSurface(img_field);

    targetTexture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888,
                                      SDL_TEXTUREACCESS_TARGET, 1024, 384);

    // 渲染第一次拼接后的图片到目标纹理
    SDL_SetRenderTarget(renderer, targetTexture);
    SDL_RenderClear(renderer);

    SDL_Rect originalRect = {0, 0, 512, 384};
    SDL_Rect flippedRect = {512, 0, 512, 384};
    SDL_RenderCopy(renderer, texture, nullptr, &originalRect); // 原图左半部分
    SDL_RenderCopyEx(renderer, texture, nullptr, &flippedRect, 0, nullptr,
                     SDL_FLIP_HORIZONTAL); // 水平翻转右半部分

    // 创建内存纹理
    memoryTexture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888,
                                      SDL_TEXTUREACCESS_TARGET, WIDTH, HEIGHT);

    // 将翻转拼接后的图像进行第二次操作并渲染到内存纹理
    SDL_SetRenderTarget(renderer, memoryTexture);
    SDL_RenderClear(renderer);

    SDL_Rect upperHalf = {0, 0, 1024, 360};   // 上半部分显示翻转拼接图像
    SDL_Rect lowerHalf = {0, 360, 1024, 360}; // 下半部分显示上下翻转后的图像

    SDL_RenderCopy(renderer, targetTexture, nullptr, &upperHalf); // 上半部分
    SDL_RenderCopyEx(renderer, targetTexture, nullptr, &lowerHalf, 0, nullptr,
                     SDL_FLIP_VERTICAL); // 下半部分翻转

    // 恢复默认渲染目标
    SDL_SetRenderTarget(renderer, nullptr);

    // 显示背景在窗口中
    SDL_RenderClear(renderer);
    SDL_RenderCopy(renderer, memoryTexture, nullptr, nullptr);
    SDL_RenderPresent(renderer);
  }

  ~Sim2D() {
    TTF_CloseFont(font);
    TTF_Quit();
    SDL_DestroyTexture(memoryTexture);
    SDL_DestroyTexture(targetTexture);
    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);

    SDL_Quit();
  }

  void DrawText(const char *text, int x, int y, SDL_Color color) {
    SDL_Surface *surface = TTF_RenderText_Blended(font, text, color);
    if (!surface) {
      printf("Failed to create text surface: %s\n", TTF_GetError());
      return;
    }

    SDL_Texture *texture = SDL_CreateTextureFromSurface(renderer, surface);
    if (!texture) {
      printf("Failed to create text texture: %s\n", SDL_GetError());
      SDL_FreeSurface(surface);
      return;
    }

    SDL_Rect textRect = {x, y, surface->w, surface->h};
    SDL_RenderCopy(renderer, texture, nullptr, &textRect);

    SDL_DestroyTexture(texture);
    SDL_FreeSurface(surface);
  }

  // 在目标位置绘制一个绿色圆圈，并用箭头表示目标角度
  void DrawRobotTarget(int x, int y, double angle) {
    // 绘制绿色圆圈表示目标位置
    SDL_SetRenderDrawColor(renderer, 255, 255, 0, 255);
    for (int w = 0; w < 360; w++) {
      double rad = w * PI / 180.0;
      int px = x + static_cast<int>(20 * cos(rad)); // 半径为20
      int py = y + static_cast<int>(20 * sin(rad));
      SDL_RenderDrawPoint(renderer, px, py);
    }

    // 绘制蓝色箭头表示目标角度
    SDL_SetRenderDrawColor(renderer, 255, 0, 255, 255);
    int arrowLength = 30; // 箭头长度
    int endX = x + static_cast<int>(arrowLength * cos(angle));
    int endY = y + static_cast<int>(arrowLength * sin(angle));
    SDL_RenderDrawLine(renderer, x, y, endX, endY);

    // 添加箭头末端的小短线（形成箭头效果）
    double arrowAngle1 = angle + PI / 6; // 偏移30度
    double arrowAngle2 = angle - PI / 6; // 偏移-30度
    int arrowEndX1 = endX - static_cast<int>(10 * cos(arrowAngle1));
    int arrowEndY1 = endY - static_cast<int>(10 * sin(arrowAngle1));
    int arrowEndX2 = endX - static_cast<int>(10 * cos(arrowAngle2));
    int arrowEndY2 = endY - static_cast<int>(10 * sin(arrowAngle2));

    SDL_RenderDrawLine(renderer, endX, endY, arrowEndX1, arrowEndY1);
    SDL_RenderDrawLine(renderer, endX, endY, arrowEndX2, arrowEndY2);
  }

  // 在指定位置绘制一个机器人，显示其角度方向
  void DrawRobot(int x, int y, double angle) {
    // 绘制红色圆形表示机器人主体
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    for (int w = 0; w < 360; w++) {
      double rad = w * PI / 180.0;
      int px = x + static_cast<int>(30 * cos(rad)); // 半径为10
      int py = y + static_cast<int>(30 * sin(rad));
      SDL_RenderDrawPoint(renderer, px, py);
    }

    // 绘制绿色线段表示机器人方向
    SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
    int lineLength = 45; // 线段长度
    int endX = x + static_cast<int>(lineLength * cos(angle));
    int endY = y + static_cast<int>(lineLength * sin(angle));
    SDL_RenderDrawLine(renderer, x, y, endX, endY);
  }

  // 在(x, y)绘制一个蓝色圆形，表示一个小球
  /******  af50264b-a1c7-4c44-8b27-6df8e299b481  *******/ void DrawBall(int x,
                                                                        int y) {
    SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
    for (int w = 0; w < 360; w++) {
      double rad = w * PI / 180.0;
      int px = x + static_cast<int>(15 * cos(rad)); // 半径为10
      int py = y + static_cast<int>(15 * sin(rad));
      SDL_RenderDrawPoint(renderer, px, py);
    }
  }

  // 更新渲染，以之前的图片为背景
  void Update() {
    SetRobot(NaoRobot::nao_robot->robot_pos.x(),
             NaoRobot::nao_robot->robot_pos.y(),
             NaoRobot::nao_robot->robot_angle.yaw_);
    SetBall(NaoRobot::nao_robot->ball_pos.x(),
            NaoRobot::nao_robot->ball_pos.y());
    SetRobotTargetAngle(NaoRobot::nao_robot->target_angle);
    SetRobotTargetPos(NaoRobot::nao_robot->target_x,
                      NaoRobot::nao_robot->target_y);

    // 清空当前渲染器
    SDL_RenderClear(renderer);

    // 渲染背景
    SDL_RenderCopy(renderer, memoryTexture, nullptr, nullptr);

    SDL_Color white = {255, 255, 255, 255};
    DrawText(
        magic_enum::enum_name((NaoRobot::nao_robot->current_motion)).data(), 10,
        10, white);

    // 绘制机器人
    DrawRobot(static_cast<int>(robot.x * WIDTH),
              static_cast<int>(robot.y * HEIGHT), robot.angle);

    // 绘制球
    DrawBall(static_cast<int>(ball.x * WIDTH),
             static_cast<int>(ball.y * HEIGHT));

    printf("robot.target_x, robot.target_y: %f, %f\n", robot.target_x,
           robot.target_y);

    printf("ball.x, ball.y: %f, %f\n", ball.x, ball.y);

    DrawRobotTarget(static_cast<int>(robot.target_x * WIDTH),
                    static_cast<int>(robot.target_y * HEIGHT),
                    robot.target_angle);

    // 显示所有内容
    SDL_RenderPresent(renderer);

    SDL_Delay(10);
  }

  void SetRobot(double x, double y, double angle) {
    robot.x = x;
    robot.y = y;
    robot.angle = angle;
  }

  void SetBall(double x, double y) { ball.x = x, ball.y = y; }

  void SetRobotTargetPos(double x, double y) {
    robot.target_x = x;
    robot.target_y = y;
  }

  void SetRobotTargetAngle(double angle) { robot.target_angle = angle; }

  SDL_Texture *texture;
  SDL_Texture *targetTexture;
  SDL_Texture *memoryTexture;
  SDL_Renderer *renderer;
  SDL_Window *window;
  TTF_Font *font;

  struct {
    double x = 0, y = 0, angle = 0;
    double target_x = 0, target_y = 0;
    double target_angle = 0;
  } robot;

  struct {
    double x = 0, y = 0;
  } ball;

  LibXR::Thread thread;

  static Sim2D *sim2d;
};
