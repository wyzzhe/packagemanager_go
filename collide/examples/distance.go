// +build ignore

package main

import (
	"fmt"
	"image/color"
	"math"

	"git.sr.ht/~adnano/go-collide"
	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/ebitenutil"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

// Body represents a collide body.
type Body struct {
	Position        collide.Point // The position of the body.
	Rotation        float64       // The rotation of the body in radians.
	Shape           collide.Shape // The shape of the body.
	Velocity        collide.Point // The velocity of the body.
	AngularVelocity float64       // The angular velocity of the body.
	Color           color.NRGBA   // The color of the body.
}

func (b Body) Transform() collide.Transform {
	return collide.NewTransform(b.Position, b.Rotation)
}

func (b *Body) Advance(dt float64) {
	b.Rotation += b.AngularVelocity * dt
	b.Rotation = math.Mod(b.Rotation, 2*math.Pi)
	b.Position = b.Position.Add(b.Velocity.Mul(dt))
}

type Example struct {
	a, b Body
	t    float64
}

func (e *Example) Layout(w, h int) (int, int) {
	return 180, 160
}

func (e *Example) Update() error {
	const dt = 1.0 / 60.0

	cx, cy := ebiten.CursorPosition()
	e.a.Position = collide.Point{float64(cx), float64(cy)}
	e.a.Advance(dt)

	// Calculate time of impact
	sweepA := collide.Sweep{
		P0: e.a.Position,
		P1: e.a.Position,
		R0: e.a.Rotation,
		R1: e.a.Rotation,
	}
	sweepB := collide.Sweep{
		P0: e.b.Position,
		P1: e.b.Position.Add(e.b.Velocity.Mul(dt)),
		R0: e.b.Rotation,
		R1: e.b.Rotation + e.b.AngularVelocity*dt,
	}
	simplex := &collide.Simplex{}
	t := collide.TimeOfImpact(simplex, e.b.Shape, sweepB, e.a.Shape, sweepA)
	e.t = t

	sweepB = sweepB.Advance(t)
	e.b.Position = sweepB.P0
	e.b.Position.X = math.Mod(e.b.Position.X, 180)
	e.b.Rotation = sweepB.R0

	// Resolve collisions
	// collision := collide.Collide(e.a.Shape, e.a.Transform(), e.b.Shape, e.b.Transform())
	// if collision != nil {
	// 	e.a.Position = e.a.Position.Sub(collision.Normal.Mul(collision.Depth))
	// }

	return nil
}

func (e *Example) Draw(dst *ebiten.Image) {
	sa, sb := e.a.Shape, e.b.Shape
	xfa, xfb := e.a.Transform(), e.b.Transform()

	drawShape(dst, sa, xfa, e.a.Color)
	drawShape(dst, sb, xfb, e.b.Color)

	distance := collide.Distance(sa, xfa, sb, xfb)
	ebitenutil.DebugPrint(dst, fmt.Sprintf("Distance: %.2f", distance))
	ebitenutil.DebugPrint(dst, fmt.Sprintf("\nTime of Impact: %.2f", e.t))

	collision := collide.Collide(sa, xfa, sb, xfb)
	if collision != nil {
		ebitenutil.DebugPrint(dst,
			fmt.Sprintf("\n\nCollision Normal: (%.2f, %.2f)\nCollision Depth: %.2f",
				collision.Normal.X, collision.Normal.Y, collision.Depth))
	}
}

func drawShape(dst *ebiten.Image, shape collide.Shape, xf collide.Transform, color color.NRGBA) {
	switch shape := shape.(type) {
	case *collide.Circle:
		drawCircle(dst, shape, xf, color)
	case *collide.Polygon:
		drawPolygon(dst, shape, xf, color)
	}
}

func drawCircle(dst *ebiten.Image, circle *collide.Circle, xf collide.Transform, color color.NRGBA) {
	center := xf.Mul(circle.Center)
	cx, cy := float32(center.X), float32(center.Y)
	r := float32(circle.Radius)

	const k = 0.551915024494
	var path vector.Path
	path.MoveTo(cx, cy+r)
	path.CubicTo(cx+k*r, cy+r, cx+r, cy+k*r, cx+r, cy)
	path.CubicTo(cx+r, cy-k*r, cx+k*r, cy-r, cx, cy-r)
	path.CubicTo(cx-k*r, cy-r, cx-r, cy-k*r, cx-r, cy)
	path.CubicTo(cx-r, cy+k*r, cx-k*r, cy+r, cx, cy+r)

	op := &vector.FillOptions{}
	op.Color = color
	path.Fill(dst, op)
}

func drawPolygon(dst *ebiten.Image, poly *collide.Polygon, xf collide.Transform, color color.NRGBA) {
	var path vector.Path
	for i, p := range poly.Points {
		p = xf.Mul(p)
		if i == 0 {
			path.MoveTo(float32(p.X), float32(p.Y))
		} else {
			path.LineTo(float32(p.X), float32(p.Y))
		}
	}

	op := &vector.FillOptions{}
	op.Color = color
	path.Fill(dst, op)
}

func main() {
	test := Example{
		a: Body{
			Position:        collide.Point{50, 50},
			Shape:           collide.Rect(0, 0, 10, 10),
			Color:           color.NRGBA{255, 0, 0, 127},
			AngularVelocity: 0,
		},
		b: Body{
			Position:        collide.Point{70, 70},
			Shape:           collide.Rect(0, 0, 30, 20),
			Color:           color.NRGBA{0, 0, 255, 127},
			Velocity:        collide.Point{100, 0},
			AngularVelocity: 0,
		},
	}
	ebiten.SetWindowTitle("collide test")
	ebiten.SetWindowSize(640, 480)
	ebiten.SetWindowResizable(true)
	if err := ebiten.RunGame(&test); err != nil {
		panic(err)
	}
}
