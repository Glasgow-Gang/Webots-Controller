#include <SDL.h>
#include <SDL_image.h>
#include <unistd.h>

#define HEIGHT 720
#define WIDTH 1024

class Sim2D {
public:
  Sim2D() {
    SDL_Init(SDL_INIT_EVERYTHING);
    IMG_Init(IMG_INIT_JPG | IMG_INIT_PNG);

    window = SDL_CreateWindow("Example", SDL_WINDOWPOS_UNDEFINED,
                              SDL_WINDOWPOS_UNDEFINED, WIDTH, HEIGHT,
                              SDL_WINDOW_SHOWN);

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    auto img_field = IMG_Load("../field.jpg");
    if (!img_field) {
      printf("Failed to load image: %s\n", IMG_GetError());
      SDL_Quit();
      exit(-1);
    }

    texture = SDL_CreateTextureFromSurface(renderer, img_field);
    SDL_FreeSurface(img_field);

    // 创建一个目标纹理用于合并后的图像
    targetTexture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888,
                                      SDL_TEXTUREACCESS_TARGET, 1024, 384);

    // 设置目标纹理为渲染目标
    SDL_SetRenderTarget(renderer, targetTexture);

    // 清空目标纹理
    SDL_RenderClear(renderer);

    // 定义左右拼接的矩形
    SDL_Rect originalRect = {0, 0, 512, 384};  // 左侧显示原始图像
    SDL_Rect flippedRect = {512, 0, 512, 384}; // 右侧显示水平翻转图像

    // 渲染到目标纹理
    SDL_RenderCopy(renderer, texture, nullptr, &originalRect);
    SDL_RenderCopyEx(renderer, texture, nullptr, &flippedRect, 0, nullptr,
                     SDL_FLIP_HORIZONTAL);

    // 恢复默认渲染目标（窗口）
    SDL_SetRenderTarget(renderer, nullptr);

    // 将目标纹理的内容复制到内存纹理中
    memoryTexture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888,
                                      SDL_TEXTUREACCESS_TARGET, 1024, 384);
    SDL_SetRenderTarget(renderer, memoryTexture);
    SDL_RenderCopy(renderer, targetTexture, nullptr, nullptr);

    // 清空窗口渲染器
    SDL_SetRenderTarget(renderer, nullptr);
    SDL_RenderClear(renderer);

    // 定义窗口区域
    SDL_Rect upperHalf = {0, 0, 1024, 360};   // 上半部分显示合并纹理
    SDL_Rect lowerHalf = {0, 360, 1024, 360}; // 下半部分显示翻转后的纹理

    // 渲染到上半部分（未翻转）
    SDL_RenderCopy(renderer, memoryTexture, nullptr, &upperHalf);

    // 渲染到下半部分（翻转后）
    SDL_RenderCopyEx(renderer, memoryTexture, nullptr, &lowerHalf, 0, nullptr,
                     SDL_FLIP_VERTICAL);

    // 显示渲染结果
    SDL_RenderPresent(renderer);

    // 等待退出
    pause();
  }

  ~Sim2D() {
    SDL_DestroyTexture(memoryTexture);
    SDL_DestroyTexture(targetTexture);
    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);

    SDL_Quit();
  }

  SDL_Texture *texture;
  SDL_Texture *targetTexture;
  SDL_Texture *memoryTexture; // 用于存储图像数据的内存纹理
  SDL_Renderer *renderer;
  SDL_Window *window;
};
