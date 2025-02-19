package collide

import (
	"math"
)

type axis int

const (
	axisPoints axis = iota
	axisFaceA
	axisFaceB
)

// separation represents a separation function.
type separation struct {
	kind   axis
	shapeA Shape // proxy for shape A
	shapeB Shape // proxy for shape B
	axis   Point // separating axis
	local  Point // local point
}

func newSeparation(shapeA, shapeB Shape) *separation {
	return &separation{
		shapeA: shapeA,
		shapeB: shapeB,
	}
}

// init initializes the separation function.
func (s *separation) init(cache *SimplexCache, xfa, xfb Transform) {
	if cache.count == 1 {
		// One point on A and one on B.
		s.kind = axisPoints

		indexA := cache.indexA[0]
		pointA := s.shapeA.getVertex(indexA)
		pointA = xfa.Mul(pointA)

		indexB := cache.indexB[0]
		pointB := s.shapeB.getVertex(indexB)
		pointB = xfb.Mul(pointB)

		s.axis = pointB.Sub(pointA).Normalize()
	} else if cache.indexA[0] == cache.indexA[1] {
		// Two points on B and one on A.
		s.kind = axisFaceB

		indexB1 := cache.indexB[0]
		indexB2 := cache.indexB[1]
		pointB1 := s.shapeB.getVertex(indexB1)
		pointB2 := s.shapeB.getVertex(indexB2)
		s.axis = CrossPS(pointB2.Sub(pointB1).Normalize(), 1.0)
		s.local = pointB1.Add(pointB2).Mul(0.5)

		pointB := xfb.Mul(s.local)
		indexA := cache.indexA[0]
		pointA := s.shapeA.getVertex(indexA)
		pointA = xfa.Mul(pointA)

		normal := xfb.Rotation.Mul(s.axis)
		dist := Dot(pointA.Sub(pointB), normal)
		if dist < 0 {
			s.axis = s.axis.Neg()
		}
	} else {
		// Two points on A and one or two points on B.
		s.kind = axisFaceA

		indexA1 := cache.indexA[0]
		indexA2 := cache.indexA[1]
		pointA1 := s.shapeA.getVertex(indexA1)
		pointA2 := s.shapeA.getVertex(indexA2)
		s.axis = CrossPS(pointA2.Sub(pointA1).Normalize(), 1.0)
		s.local = pointA1.Add(pointA2).Mul(0.5)

		pointA := xfa.Mul(s.local)
		indexB := cache.indexB[0]
		pointB := s.shapeB.getVertex(indexB)
		pointB = xfb.Mul(pointB)

		normal := xfa.Rotation.Mul(s.axis)
		dist := Dot(pointB.Sub(pointA), normal)
		if dist < 0 {
			s.axis = s.axis.Neg()
		}
	}
}

// Find the deepest point. Return the witness points and the separation between them.
func (s *separation) MinSeparation(xfa, xfb Transform) (int, int, float64) {
	switch s.kind {
	case axisPoints:
		axisA := xfa.Rotation.MulT(s.axis)
		axisB := xfb.Rotation.MulT(s.axis.Neg())

		indexA := s.shapeA.getSupport(axisA)
		pointA := s.shapeA.getVertex(indexA)
		pointA = xfa.Mul(pointA)

		indexB := s.shapeB.getSupport(axisB)
		pointB := s.shapeB.getVertex(indexB)
		pointB = xfb.Mul(pointB)

		separation := Dot(pointB.Sub(pointA), s.axis)
		return indexA, indexB, separation

	case axisFaceA:
		normal := xfa.Rotation.Mul(s.axis)
		axisB := xfb.Rotation.MulT(normal.Neg())

		indexB := s.shapeB.getSupport(axisB)
		pointB := s.shapeB.getVertex(indexB)
		pointB = xfb.Mul(pointB)
		pointA := xfa.Mul(s.local)

		separation := Dot(pointB.Sub(pointA), normal)
		return -1, indexB, separation

	case axisFaceB:
		normal := xfb.Rotation.Mul(s.axis)
		axisA := xfa.Rotation.MulT(normal.Neg())

		indexA := s.shapeA.getSupport(axisA)
		pointA := s.shapeA.getVertex(indexA)
		pointA = xfa.Mul(pointA)
		pointB := xfb.Mul(s.local)

		separation := Dot(pointA.Sub(pointB), normal)
		return indexA, -1, separation

	default:
		panic("unknown separation kind")
	}
}

// Evaluate returns the separation at time t.
func (s *separation) Evaluate(indexA int, xfa Transform, indexB int, xfb Transform) float64 {
	switch s.kind {
	case axisPoints:
		a := xfa.Mul(s.shapeA.getVertex(indexA))
		b := xfb.Mul(s.shapeB.getVertex(indexB))
		separation := Dot(b.Sub(a), s.axis)
		return separation

	case axisFaceA:
		normal := xfa.Rotation.Mul(s.axis)
		a := xfa.Mul(s.local)
		b := xfb.Mul(s.shapeB.getVertex(indexB))
		separation := Dot(b.Sub(a), normal)
		return separation

	case axisFaceB:
		normal := xfb.Rotation.Mul(s.axis)
		a := xfa.Mul(s.shapeA.getVertex(indexA))
		b := xfb.Mul(s.local)
		separation := Dot(a.Sub(b), normal)
		return separation

	default:
		panic("unknown separation kind")
	}
}

// CCD via the local separating axis method. This seeks progression
// by computing the largest time at which separation is maintained.
func TimeOfImpact(simplex *Simplex, a Shape, sweepA Sweep, b Shape, sweepB Sweep) float64 {
	const target = 0.01
	const tolerance = 0.25 * 0.005

	cache := &SimplexCache{}
	t1 := 0.0

	// The outer loop progressively attempts to compute new separating axes.
	// This loop terminates when an axis is repeated (no progress is made).
loop:
	for i := 0; i < 100; i++ {
		// Get the closest features at t1.
		xfa, xfb := sweepA.GetTransform(t1), sweepB.GetTransform(t1)
		simplex.ReadCache(cache, a, xfa, b, xfb)
		simplex.GJK(a, xfa, b, xfb)
		simplex.WriteCache(cache)
		distance := simplex.ClosestPoint().Length()

		// If the shapes are overlapped, we give up on continuous collision.
		if distance <= 0 {
			// Failure!
			return 0
		}

		if distance < target+tolerance {
			// Victory!
			break
		}

		// Initialize the separating axis.
		fcn := newSeparation(a, b)
		fcn.init(cache, xfa, xfb)

		// Compute the TOI on the separating axis. We do this by successively
		// resolving the deepest point. This loop is bounded by the number of vertices.
		t2 := 1.0
		for i := 0; i < 20; i++ {
			// Find the deepest point at t2. Store the witness points.
			var s2 float64
			xfa, xfb := sweepA.GetTransform(t2), sweepB.GetTransform(t2)
			indexA, indexB, s2 := fcn.MinSeparation(xfa, xfb)

			// Is the final configuration separated?
			if s2 > target+tolerance {
				// Victory!
				return 1.0
			}

			// Has the separation reached tolerance?
			if s2 > target-tolerance {
				// Advance the sweeps
				t1 = t2
				break
			}

			// Compute the initial separation of the witness points.
			xfa, xfb = sweepA.GetTransform(t1), sweepB.GetTransform(t1)
			s1 := fcn.Evaluate(indexA, xfa, indexB, xfb)

			// Check for initial overlap. This might happen if the root finder
			// runs out of iterations.
			if s1 < target-tolerance {
				// Failure
				break loop
			}

			// Check for touching
			if s1 <= target+tolerance {
				// Victory! t1 should hold the TOI (could be 0).
				break loop
			}

			// Compute 1D root of: f(x) - target = 0
			a1, a2 := t1, t2
			for j := 0; j < 50; j++ {
				var t float64
				if (j & 1) != 0 {
					// Secant rule to improve convergence.
					t = a1 + (target-s1)*(a2-a1)/(s2-s1)
				} else {
					// Bisection to guarantee progress.
					t = 0.5 * (a1 + a2)
				}

				xfa, xfb := sweepA.GetTransform(t), sweepB.GetTransform(t)
				s := fcn.Evaluate(indexA, xfa, indexB, xfb)
				if math.Abs(s-target) < tolerance {
					// t2 holds a tentative value for t1
					t2 = t
					break
				}

				// Ensure we continue to bracket the root.
				if s > target {
					a1 = t
					s1 = s
				} else {
					a2 = t
					s2 = s
				}
			}
		}
	}
	return t1
}
