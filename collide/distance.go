package collide

// SimplexCache is used to speed up repeated calls to distance functions.
type SimplexCache struct {
	metric float64 // length or area
	count  byte    // number of verticies
	indexA [3]int  // vertices on shape A
	indexB [3]int  // vertices on shape B
}

// vertex represents a vertex on a simplex.
type vertex struct {
	a      Point   // support point on shape A
	b      Point   // support point on shape B
	p      Point   // point on Minkowski difference (B - A)
	u      float64 // barycentric coordinate for closest point
	indexA int     // index of support point A
	indexB int     // index of support point B
}

// Simplex represents a simplex.
type Simplex struct {
	count byte      // number of vertices
	v     [3]vertex // vertices
}

// ReadCache populates the simplex from the cache.
func (s *Simplex) ReadCache(cache *SimplexCache, a Shape, xfa Transform, b Shape, xfb Transform) {
	s.count = cache.count
	for i := byte(0); i < s.count; i++ {
		v := &s.v[i]
		v.indexA = cache.indexA[i]
		v.indexB = cache.indexB[i]
		v.a = a.getVertex(v.indexA)
		v.b = b.getVertex(v.indexB)
		a := xfa.Mul(v.a)
		b := xfb.Mul(v.b)
		v.p = b.Sub(a)
	}

	// Compute the new simplex metric. If it is substantially different than
	// the old metric then flush the simplex.
	if s.count > 1 {
		mOld := cache.metric
		mNew := s.Metric()
		if mNew < 0.5*mOld || 2*mOld < mNew || mNew < 0.0001 {
			// Reset the simplex.
			s.count = 0
		}
	}
}

// WriteCache populates the cache.
func (s *Simplex) WriteCache(cache *SimplexCache) {
	cache.metric = s.Metric()
	cache.count = s.count
	for i := byte(0); i < s.count; i++ {
		cache.indexA[i] = s.v[i].indexA
		cache.indexB[i] = s.v[i].indexB
	}
}

// ClosestPoint returns the closest point on the simplex to the origin.
func (s *Simplex) ClosestPoint() Point {
	switch s.count {
	case 1:
		// p0
		return s.v[0].p
	case 2:
		// p0*u0 + p1*u1
		return s.v[0].p.Mul(s.v[0].u).Add(s.v[1].p.Mul(s.v[1].u))
	case 3:
		// origin
		return Point{}
	default:
		panic("bad simplex length")
	}
}

// WitnessPoints returns the witness points on the original shapes.
func (s *Simplex) WitnessPoints() (Point, Point) {
	switch s.count {
	case 1:
		// a0, b0
		return s.v[0].a, s.v[1].b
	case 2:
		// a0*u0 + a1*u1, b0*u0 + b1*u1
		return s.v[0].a.Mul(s.v[0].u).Add(s.v[1].a.Mul(s.v[1].u)),
			s.v[0].b.Mul(s.v[0].u).Add(s.v[1].b.Mul(s.v[1].u))
	case 3:
		// a0*u0 + a1*u1 + a2*u2
		p := s.v[0].a.Mul(s.v[0].u).
			Add(s.v[1].a.Mul(s.v[1].u)).
			Add(s.v[2].a.Mul(s.v[2].u))
		return p, p
	default:
		panic("bad simplex length")
	}
}

// searchDirection returns the direction to search in.
func (s *Simplex) searchDirection() Point {
	switch s.count {
	case 1:
		a := s.v[0].p
		return a.Neg()
	case 2:
		a := s.v[0].p
		b := s.v[1].p
		ab := b.Sub(a)
		ao := a.Neg()

		sign := Cross(ab, ao)
		if sign > 0 {
			// Origin is left of ab
			return CrossSP(1.0, ab)
		} else {
			// Origin is right of ab
			return CrossPS(ab, 1.0)
		}
	default:
		panic("bad simplex length")
	}
}

// evolve evolves the simplex.
func (s *Simplex) evolve() {
	switch s.count {
	case 1:
		return
	case 2:
		s.evolveLine()
	case 3:
		s.evolveTriangle()
	default:
		panic("bad simplex length")
	}
}

// evolveLine evolves a line segment using barycentric coordinates.
func (s *Simplex) evolveLine() {
	a := s.v[0]
	b := s.v[1]

	// Compute barycentric coordinates (pre-division)
	n := b.p.Sub(a.p)
	u := Dot(b.p, n)
	v := Dot(a.p.Neg(), n)

	// Region A
	if v <= 0 {
		// Reduce simplex to vertex A
		s.count = 1
		s.v[0] = a
		return
	}

	// Region B
	if u <= 0 {
		// Reduce simplex to vertex B
		s.count = 1
		s.v[0] = b
		return
	}

	// Region AB
	l := n.LengthSquared()
	s.v[0].u = u / l
	s.v[1].u = v / l
}

// evolveTriangle evolves a triangle using barycentric coordinates.
func (s *Simplex) evolveTriangle() {
	a := s.v[0]
	b := s.v[1]
	c := s.v[2]

	// Compute edge barycentric coordinates (pre-division)
	ab := b.p.Sub(a.p)
	bc := c.p.Sub(b.p)
	ca := a.p.Sub(c.p)
	uAB, vAB := Dot(b.p, ab), Dot(a.p.Neg(), ab)
	uBC, vBC := Dot(c.p, bc), Dot(b.p.Neg(), bc)
	uCA, vCA := Dot(a.p, ca), Dot(c.p.Neg(), ca)

	// Region A
	if vAB <= 0 && uCA <= 0 {
		// Reduce simplex to vertex A
		s.count = 1
		s.v[0] = a
		return
	}
	// Region B
	if uAB <= 0 && vBC <= 0 {
		// Reduce simplex to vertex B
		s.count = 1
		s.v[0] = b
		return
	}
	// Region C
	if uBC <= 0 && vCA <= 0 {
		// Reduce simplex to vertex C
		s.count = 1
		s.v[0] = c
	}

	// Compute signed triangle area
	area := Cross(b.p.Sub(a.p), c.p.Sub(a.p))

	// Compute triangle barycentric coordinates (pre-division)
	uABC := Cross(b.p, c.p) * area
	vABC := Cross(c.p, a.p) * area
	wABC := Cross(a.p, b.p) * area

	// Region AB
	if uAB > 0 && vAB > 0 && wABC <= 0 {
		// Reduce simplex to AB
		l := ab.LengthSquared()
		a.u = uAB / l
		b.u = vAB / l

		s.count = 2
		s.v[0] = a
		s.v[1] = b
		return
	}
	// Region BC
	if uBC > 0 && vBC > 0 && uABC <= 0 {
		// Reduce simplex to BC
		l := bc.LengthSquared()
		b.u = uBC / l
		c.u = vBC / l

		s.count = 2
		s.v[0] = b
		s.v[1] = c
		return
	}
	// Region CA
	if uCA > 0 && vCA > 0 && vABC <= 0 {
		// Reduce simplex to CA
		l := ca.LengthSquared()
		c.u = uCA / l
		a.u = vCA / l

		s.count = 2
		s.v[0] = c
		s.v[1] = a
		return
	}

	// Region ABC
	// The triangle area is guaranteed to be non-zero
	d := uABC + vABC + wABC
	s.v[0].u = uABC / d
	s.v[1].u = vABC / d
	s.v[2].u = wABC / d
}

// Metric returns a metric that identifies this simplex.
func (s *Simplex) Metric() float64 {
	switch s.count {
	case 1:
		return 0
	case 2:
		// length
		a := s.v[0]
		b := s.v[1]
		return a.p.Sub(b.p).Length()
	case 3:
		// area
		a := s.v[0]
		b := s.v[1]
		c := s.v[2]
		return Cross(b.p.Sub(a.p), c.p.Sub(a.p))
	default:
		panic("bad simplex length")
	}
}

// GJK implements the Gilbert-Johnson-Keerthi distance algorithm.
func (s *Simplex) GJK(a Shape, xfa Transform, b Shape, xfb Transform) {
	// If the simplex is empty or invalid
	if s.count == 0 {
		// Pick arbitrary initial simplex.
		indexA := 0
		indexB := 0
		va := a.getVertex(indexA)
		vb := b.getVertex(indexB)

		s.count = 1
		s.v[0] = vertex{
			a:      va,
			b:      vb,
			p:      xfb.Mul(vb).Sub(xfa.Mul(va)),
			u:      1,
			indexA: indexA,
			indexB: indexB,
		}
	}

	var oldCount byte
	var oldIndexA [3]int
	var oldIndexB [3]int

loop:
	for {
		// Store old vertices to check for duplicates later on.
		oldCount = s.count
		for i := byte(0); i < s.count; i++ {
			oldIndexA[i] = s.v[i].indexA
			oldIndexB[i] = s.v[i].indexB
		}

		// Evolve the simplex.
		s.evolve()

		// If we have three points, then the origin is in the corresponding triangle.
		if s.count == 3 {
			break
		}

		// Get search direction.
		dir := s.searchDirection()
		if dir.LengthSquared() == 0 {
			break
		}

		// Calculate a new support point in the search direction.
		indexA := a.getSupport(xfa.Rotation.MulT(dir.Neg()))
		indexB := b.getSupport(xfb.Rotation.MulT(dir))
		va := a.getVertex(indexA)
		vb := b.getVertex(indexB)

		support := vertex{
			a:      va,
			b:      vb,
			p:      xfb.Mul(vb).Sub(xfa.Mul(va)),
			u:      1,
			indexA: indexA,
			indexB: indexB,
		}

		// Check for duplicate support points. This is the main termination criteria.
		for i := byte(0); i < oldCount; i++ {
			if support.indexA == oldIndexA[i] && support.indexB == oldIndexB[i] {
				break loop
			}
		}

		// Add the new support point.
		s.v[s.count] = support
		s.count++
	}
}

// Distance returns the distance between a and b.
func Distance(a Shape, xfa Transform, b Shape, xfb Transform) float64 {
	var simplex Simplex
	simplex.GJK(a, xfa, b, xfb)
	// The distance between the shapes is equal to the distance
	// between the Minkowski difference and the origin.
	return simplex.ClosestPoint().Length()
}
