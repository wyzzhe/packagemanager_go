package collide

import "math"

// A Point is an X, Y coordinate pair.
type Point struct {
	X, Y float64
}

// Pt is shorthand for Point{X, Y}.
func Pt(X, Y float64) Point {
	return Point{X, Y}
}

// Add returns the vector p+q.
func (p Point) Add(q Point) Point {
	return Point{p.X + q.X, p.Y + q.Y}
}

// Sub returns the vector p-q.
func (p Point) Sub(q Point) Point {
	return Point{p.X - q.X, p.Y - q.Y}
}

// Mul returns the vector p*k.
func (p Point) Mul(k float64) Point {
	return Point{p.X * k, p.Y * k}
}

// Div returns the vector p/k.
func (p Point) Div(k float64) Point {
	return Point{p.X / k, p.Y / k}
}

// Neg returns the vector -p.
func (p Point) Neg() Point {
	return Point{-p.X, -p.Y}
}

// Length returns the length of the vector p.
func (p Point) Length() float64 {
	return math.Sqrt(p.X*p.X + p.Y*p.Y)
}

// LengthSquared returns the squared length of the vector p.
func (p Point) LengthSquared() float64 {
	return p.X*p.X + p.Y*p.Y
}

// Normalize returns the unit vector in the direction of p.
func (p Point) Normalize() Point {
	l := p.Length()
	if l == 0 {
		return p
	}
	return p.Div(l)
}

// IsZero reports whether the point has zero length.
func (p Point) IsZero() bool {
	return p.X == 0 && p.Y == 0
}

// Dot returns the dot product of p and q.
func Dot(p, q Point) float64 {
	return p.X*q.X + p.Y*q.Y
}

// Cross returns the cross product of p and q.
func Cross(p, q Point) float64 {
	return p.X*q.Y - p.Y*q.X
}

func CrossPS(p Point, s float64) Point {
	return Point{s * p.Y, -s * p.X}
}

func CrossSP(s float64, p Point) Point {
	return Point{-s * p.Y, s * p.X}
}

// Rotation represents a rotation.
type Rotation struct {
	Sin, Cos float64
}

// NewRotation returns a new rotation of theta radians.
func NewRotation(theta float64) Rotation {
	sin, cos := math.Sincos(theta)
	return Rotation{sin, cos}
}

// Mul returns the vector p rotated.
func (r Rotation) Mul(p Point) Point {
	return Point{
		p.X*r.Cos - p.Y*r.Sin,
		p.X*r.Sin + p.Y*r.Cos,
	}
}

// MulT returns the vector p inverse rotated.
func (r Rotation) MulT(p Point) Point {
	return Point{
		p.X*r.Cos + p.Y*r.Sin,
		p.Y*r.Cos - p.X*r.Sin,
	}
}

// A Transform represents translation and rotation.
type Transform struct {
	Position Point
	Rotation Rotation
}

// NewTransform returns a Transform with the given position and rotation.
func NewTransform(position Point, rotation float64) Transform {
	return Transform{
		Position: position,
		Rotation: NewRotation(rotation),
	}
}

// Mul returns the transformation of the vector p.
func (t Transform) Mul(p Point) Point {
	return t.Rotation.Mul(p).Add(t.Position)
}

// MulT returns the inverse transformation of the vector p.
func (t Transform) MulT(p Point) Point {
	return t.Rotation.MulT(p.Sub(t.Position))
}

// A Sweep interpolates between two positions and orientations.
type Sweep struct {
	P0, P1 Point   // position
	R0, R1 float64 // rotation
}

// GetTransform returns the transform at time t.
func (s Sweep) GetTransform(t float64) Transform {
	return NewTransform(
		s.P0.Mul(1-t).Add(s.P1.Mul(t)),
		s.R0*(1-t)+s.R1*t,
	)
}

// Advance advances the position and rotation to time t.
func (s Sweep) Advance(t float64) Sweep {
	return Sweep{
		P0: s.P0.Mul(1 - t).Add(s.P1.Mul(t)),
		P1: s.P1,
		R0: s.R0*(1-t) + s.R1*t,
		R1: s.R1,
	}
}
