# ros-playground

Relevant commands:

```
# run circle_visualizer with debug log level
ros2 run circle_visualizer circle_visualizer --ros-args --log-level debug

# run circle_visualizer with non-default window settings
ros2 run circle_visualizer circle_visualizer --ros-args -p window_width:=2000 -p window_height:=2000

# publish renderable circle for circle_visualizer
ros2 topic pub /renderable_circles shape_interfaces/msg/RenderableCircle "{id: a, color: {r: 255.0, g: 0.0, b: 0.0, a: 255.0}, radius: 100.0, position: {x: 500, y: 400}}" -t 1
```