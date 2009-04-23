#include <unistd.h>
#include <cstdio>
#include <cassert>
#include <ros/time.h>
#include "uvc_cam/uvc_cam.h"
#include "SDL/SDL.h"

const unsigned WIDTH = 640, HEIGHT = 480;

void save_photo(uint8_t *frame)
{
  static int image_num = 0;
  char fnbuf[500];
  snprintf(fnbuf, sizeof(fnbuf), "image%06d.ppm", image_num++);
  printf("saving %s\n", fnbuf);
  FILE *f = fopen(fnbuf, "wb");
  if (!f)
  {
    printf("couldn't open %s\n", fnbuf);
    return;
  }
  uint8_t *bgr = new uint8_t[WIDTH*HEIGHT*3];
  for (uint32_t y = 0; y < HEIGHT; y++)
    for (uint32_t x = 0; x < WIDTH; x++)
    {
      uint8_t *p = frame + y * WIDTH * 3 + x * 3;
      uint8_t *q = bgr   + y * WIDTH * 3 + x * 3;
      q[0] = p[2]; q[1] = p[1]; q[2] = p[0];
    }
  fprintf(f, "P6\n%d %d\n255\n", WIDTH, HEIGHT);
  fwrite(bgr, 1, WIDTH * HEIGHT * 3, f);
  delete[] bgr;
  fclose(f);
}

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    fprintf(stderr, "usage: view DEVICE\n");
    return 1;
  }

  uvc_cam::Cam cam(argv[1]);
  if (SDL_Init(SDL_INIT_VIDEO) < 0)
  {
    fprintf(stderr, "sdl init error: %s\n", SDL_GetError());
    return 1;
  }
  atexit(SDL_Quit);
  SDL_Surface *surf = SDL_SetVideoMode(WIDTH, HEIGHT, 24, SDL_HWSURFACE);
  assert(surf);
  SDL_WM_SetCaption("hello world", "hello world");
  ros::Time t_prev(ros::Time::now());
  int count = 0;
  for (bool done = false; !done;)
  {
    unsigned char *frame = NULL;
    uint32_t bytes_used;
    int buf_idx = cam.grab(&frame, bytes_used);
    if (count++ % 30 == 0)
    {
      ros::Time t(ros::Time::now());
      ros::Duration d(t - t_prev);
      printf("%.1f fps\n", 30.0 / d.toSec());
      t_prev = t;
    }
    if (frame)
    {
      memcpy(surf->pixels, frame, WIDTH*HEIGHT*3);
      cam.release(buf_idx);
    }
    //usleep(1000);
    SDL_UpdateRect(surf, 0, 0, WIDTH, HEIGHT);
    SDL_Event event;
    while (SDL_PollEvent(&event))
    {
      switch (event.type)
      {
        case SDL_KEYDOWN:
          switch(event.key.keysym.sym)
          {
            case SDLK_ESCAPE: done = true; break;
            case SDLK_SPACE:  save_photo(frame); break;
            default: break;
          }
          break;
        case SDL_QUIT:
          done = true;
          break;
      }
    }
  }
  return 0;
}

