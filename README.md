# ros-playground

Relevant commands:

```
ros2 run circle_visualizer circle_visualizer --ros-args --log-level debug

ros2 topic pub /renderable_circles shape_interfaces/msg/RenderableCircle "{id: a, color: {r: 255.0, g: 0.0, b: 0.0, a: 255.0}, radius: 100.0, position: {x: 500, y: 400}}" -t 1
```