#ifndef DRAWING_AREA_H
#define DRAWING_AREA_H

#include <FL/Fl_Widget.H>

#include <string>
#include <vector>

#include "park_env.h"
namespace park {
class DrawingArea : public Fl_Widget {
 public:
  DrawingArea(int X, int Y, int W, int H, const char* L = 0);

  void setEnvironment(Environment* env);
  void setParkingMap(ParkingMap map);
  void setCurrentFrame(const Frame& frame);

 protected:
  void draw() override;

 private:
  Environment* env;
  ParkingMap parkingMap;
  Frame frame;
  bool staticElementsDrawn;

  void drawStaticElements();
  void drawFrame();
};

}  // namespace park

#endif  // DRAWING_AREA_H
