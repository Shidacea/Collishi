# Collishi

Collishi is a fast 2D collision library for C++ supporting several primitive shapes.

# How to include Collishi

To use Collishi, the file "Collisions.h" needs to be included. 
A compiler with C++17 is required and the standard library needs to be used for the `std::minmax` function.

If you do not want to use the C++ standard library, you need to implement a minmax-function yourself,
which behaves exactly as `std::minmax`. Then, you need to set a definition to your own minmax function:

```c++
#define COLLISHI_MINMAX_FUNCTION your_own_minmax_function
```

# Supported shapes

Here is a list with all available shapes and their parameters.
Note that each shape has the coordinates x and y.

Point:
* x, y: Coordinates of the point

Line segment:
* x, y: Start point of the segment
* dx, dy: Coordinates of the end point, relative to (x, y)

Circle:
* x, y: Midpoint of the circle
* r: Radius of the circle

Axis aligned box:
* x, y: Any corner of the box
* w, h: Total width and height of the box

Triangle:
* x, y: Arbitrary point
* sxa, sya: Coordinates of the second point, relative to (x, y)
* sxb, syb: Coordinates of the third point, relative to (x, y)

# Usage

Each function requires a set of floating point values as arguments, in order of the shapes.
The arguments are ordered according to the list above.

For example, the collision between a circle and a triangle can be done as follows:

```c++
// Circle
float x1 = 0.0f;
float y1 = 0.0f;

float r = 1.0f;

// Triangle
float x2 = 3.0f;
float y2 = 2.0f;

float sxa2 = -1.0f;
float sya2 = -5.0f;

float sxb2 = -5.0f;
float syb2 = -1.0f;

// Get result
bool result = Collishi::collision_circle_triangle(
	x1, y1, r1, 
	x2, y2, sxa2, sya2, sxb2, syb2
);
```

In this case, the result should be `true`.

# Further information

Collishi makes heavy use of assertions to ensure that each collision routine works as intended.
This is however not a definitive guarantee and errors may still exist.
If you find an error with one of the collision routines, please submit the exact parameters in an issue.

If assertions fail for some reason or take too long to compile, you can define the value `COLLISHI_IGNORE_STATIC_ASSERTIONS`
to ignore the assertions, although this is not recommended, especially if you do this because of failing assertions.
Please submit an issue if you encounter a problem with the assertions.