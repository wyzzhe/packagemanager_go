// Based on ImpulseEngine/Collision.cpp
//
// Copyright (c) 2013 Randy Gaul http://RandyGaul.net
//
// This software is provided 'as-is', without any express or implied
// warranty. In no event will the authors be held liable for any damages
// arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
//   1. The origin of this software must not be misrepresented; you must not
//      claim that you wrote the original software. If you use this software
//      in a product, an acknowledgment in the product documentation would be
//      appreciated but is not required.
//   2. Altered source versions must be plainly marked as such, and must not be
//      misrepresented as being the original software.
//   3. This notice may not be removed or altered from any source distribution.

package collide

import (
	"math"
)

// Collision represents a collision.
type Collision struct {
	Normal Point
	Depth  float64
}

// Collide calculates a collision for two shapes.
func Collide(a Shape, xfa Transform, b Shape, xfb Transform) *Collision {
	switch a := a.(type) {
	case *Circle:
		switch b := b.(type) {
		case *Circle:
			return CollideCircles(a, xfa, b, xfb)
		case *Polygon:
			return CollideCircleAndPolygon(a, xfa, b, xfb)
		}
	case *Polygon:
		switch b := b.(type) {
		case *Circle:
			return CollidePolygonAndCircle(a, xfa, b, xfb)
		case *Polygon:
			return CollidePolygons(a, xfa, b, xfb)
		}
	}
	return nil
}

func CollideCircles(a *Circle, xfa Transform, b *Circle, xfb Transform) *Collision {
	centerA, centerB := xfa.Position.Add(a.Center), xfb.Position.Add(b.Center)
	n := centerB.Sub(centerA)
	r := a.Radius + b.Radius

	d := n.LengthSquared()
	if d > r*r {
		return nil
	}

	if d != 0 {
		n = n.Div(d)
	} else {
		// Circles are at the exact same position
		// Choose arbitrary normal
		n = Point{1, 0}
	}

	d = math.Sqrt(d)

	return &Collision{
		Normal: n,
		Depth:  d - r,
	}
}

func CollidePolygonAndCircle(a *Polygon, xfa Transform, b *Circle, xfb Transform) *Collision {
	// Compute circle position in the frame of the polygon
	center := xfa.MulT(b.Center.Add(xfb.Position))

	// Find edge with minimum penetration
	var normalIndex int
	separation := -math.MaxFloat64
	for i := range a.Points {
		s := Dot(a.Normals[i], center.Sub(a.Points[i]))

		if s > b.Radius {
			// Early out
			return nil
		}

		if s > separation {
			separation = s
			normalIndex = i
		}
	}

	// Grab face's vertices
	i := normalIndex
	j := i + 1
	if j == len(a.Points) {
		j = 0
	}
	v1, v2 := a.Points[i], a.Points[j]

	// If the center is inside the polygon
	if separation == 0 {
		normal := xfa.Rotation.Mul(a.Normals[normalIndex]).Neg()
		return &Collision{
			Normal: normal,
			Depth:  b.Radius,
		}
	}

	// Compute barycentric coordinates
	u1 := Dot(center.Sub(v1), v2.Sub(v1))
	u2 := Dot(center.Sub(v2), v1.Sub(v2))
	if u1 <= 0 {
		// Closest to v1
		d := center.Sub(v1)
		if Dot(d, d) > b.Radius*b.Radius {
			return nil
		}

		n := v1.Sub(center)
		n = xfa.Rotation.Mul(n).Normalize()
		return &Collision{
			Normal: n,
			Depth:  b.Radius - separation,
		}
	} else if u2 <= 0 {
		// Closest to v2
		n := center.Sub(v2)
		if Dot(n, n) > b.Radius*b.Radius {
			return nil
		}
		n = xfa.Rotation.Mul(n).Normalize()
		return &Collision{
			Normal: n,
			Depth:  b.Radius - separation,
		}

	} else {
		// Closest to face
		n := a.Normals[normalIndex]
		face := v1.Add(v2).Mul(0.5)
		if Dot(center.Sub(face), n) > b.Radius {
			return nil
		}

		n = xfa.Rotation.Mul(n)
		return &Collision{
			Normal: n,
			Depth:  b.Radius - separation,
		}
	}

	return nil
}

func CollideCircleAndPolygon(a *Circle, xfa Transform, b *Polygon, xfb Transform) *Collision {
	collision := CollidePolygonAndCircle(b, xfb, a, xfa)
	if collision != nil {
		collision.Normal = collision.Normal.Neg()
	}
	return collision
}

// Find the maximum separation between a and b using edge normals from a.
func findMaxSeparation(a *Polygon, xfa Transform, b *Polygon, xfb Transform) (int, float64) {
	bestIndex := 0
	maxSeparation := -math.MaxFloat64

	for i := range a.Points {
		// Retrieve a face normal from A
		n := a.Normals[i]
		n = xfa.Rotation.Mul(n)
		n = xfb.Rotation.MulT(n)
		v := a.Points[i]
		v = xfa.Mul(v)
		v = xfb.MulT(v)

		// Find deepest point
		d := math.MaxFloat64
		for j := range b.Points {
			dist := Dot(n, b.Points[j].Sub(v))
			if dist < d {
				d = dist
			}
		}

		// Store greatest distance
		if d > maxSeparation {
			maxSeparation = d
			bestIndex = i
		}
	}

	return bestIndex, maxSeparation
}

// A is the reference polygon and B is the incident polygon.
func findIncidentEdge(a *Polygon, xfa Transform, b *Polygon, xfb Transform, edge int) [2]Point {
	// Get the normal of the reference edge in B's model space
	normal := a.Normals[edge]
	normal = xfa.Rotation.Mul(normal)
	normal = xfb.Rotation.MulT(normal)

	// Find the incident edge on B
	var index int
	minDot := math.MaxFloat64
	for i := range b.Normals {
		dot := Dot(normal, b.Normals[i])
		if dot < minDot {
			minDot = dot
			index = i
		}
	}

	// Build the clip vertices for the incident edge
	i := index
	j := i + 1
	if j == len(b.Points) {
		j = 0
	}
	return [2]Point{xfb.Mul(b.Points[i]), xfb.Mul(b.Points[j])}
}

func clip(n Point, c float64, edge []Point) int {
	var sp int
	var out [2]Point
	copy(out[:], edge)

	// Retrieve distances from each endpoint to the line
	// d = ax + by - c
	d1 := Dot(n, edge[0]) - c
	d2 := Dot(n, edge[1]) - c

	// If negative (behind plane) clip
	if d1 <= 0 {
		out[sp] = edge[0]
		sp++
	}
	if d2 <= 0 {
		out[sp] = edge[1]
		sp++
	}

	// If the points are on different sides of the plane
	if d1*d2 < 0 { // less than to ignore -0.0
		// Push intersection point
		alpha := d1 / (d1 - d2)
		out[sp] = edge[0].Add(edge[1].Sub(edge[0]).Mul(alpha))
		sp++
	}

	edge[0] = out[0]
	edge[1] = out[1]
	return sp
}

func CollidePolygons(a *Polygon, xfa Transform, b *Polygon, xfb Transform) *Collision {
	// Check for a separating axis with A's edges
	edgeA, separationA := findMaxSeparation(a, xfa, b, xfb)
	if separationA >= 0 {
		return nil
	}

	// Check for a separating axis with B's edges
	edgeB, separationB := findMaxSeparation(b, xfb, a, xfa)
	if separationB >= 0 {
		return nil
	}

	var edge int  // reference edge
	var flip bool // Always point from a to b

	// Ensure that A is the reference polygon. If not, swap A and B.
	if separationB > separationA {
		a, b = b, a
		xfa, xfb = xfb, xfa
		edge = edgeB
		flip = true
	} else {
		edge = edgeA
	}

	// Find incident edge
	incidentEdge := findIncidentEdge(a, xfa, b, xfb, edge)

	//        y
	//        ^  ->n       ^
	//      +---c ------posPlane--
	//  x < | i |\
	//      +---+ c-----negPlane--
	//             \       v
	//              r
	//
	//  r : reference face
	//  i : incident poly
	//  c : clipped point
	//  n : incident normal

	// Setup reference face vertices
	i := edge
	j := i + 1
	if j == len(a.Points) {
		j = 0
	}
	v1 := a.Points[i]
	v2 := a.Points[j]

	tangent := v2.Sub(v1).Normalize()
	tangent = xfa.Rotation.Mul(tangent)
	normal := CrossPS(tangent, 1.0)

	v1 = xfa.Mul(v1)
	v2 = xfa.Mul(v2)

	refC := Dot(normal, v1)
	negSide := -Dot(tangent, v1)
	posSide := Dot(tangent, v2)

	// Clip incident face to reference face side planes
	if clip(tangent.Neg(), negSide, incidentEdge[:]) < 2 {
		// Due to floating point error, possible to not have required points
		return nil
	}
	if clip(tangent, posSide, incidentEdge[:]) < 2 {
		// Due to floating point error, possible to not have required points
		return nil
	}

	var overlap float64
	separation0 := Dot(normal, incidentEdge[0]) - refC
	if separation0 <= 0 {
		overlap = -separation0
	}
	separation1 := Dot(normal, incidentEdge[1]) - refC
	if separation1 <= 0 {
		// Maximum penetration
		overlap = math.Max(overlap, -separation1)
	}

	// Flip normal
	if flip {
		normal = normal.Neg()
	}

	if overlap == 0 {
		return nil
	}
	return &Collision{
		Normal: normal,
		Depth:  overlap,
	}
}
