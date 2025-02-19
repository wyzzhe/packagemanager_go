package collide

// Shape represents a shape.
type Shape interface {
	getSupport(dir Point) int
	getVertex(index int) Point
}

// Circle represents a circle shape.
type Circle struct {
	Center Point
	Radius float64
}

func (c *Circle) getSupport(dir Point) int {
	return 0
}

func (c *Circle) getVertex(index int) Point {
	return c.Center
}

// Polygon represents a collection of points.
type Polygon struct {
	Points  []Point
	Normals []Point
}

// NewPolygon returns a polygon with the given points specified in clockwise order.
func NewPolygon(points ...Point) *Polygon {
	// Calculate edge normals
	var normals []Point
	for i := range points {
		j := i + 1
		if j >= len(points) {
			j = 0
		}

		edge := points[j].Sub(points[i])
		normal := CrossPS(edge, 1.0).Normalize()
		normals = append(normals, normal)
	}
	return &Polygon{
		Points:  points,
		Normals: normals,
	}
}

// getSupport returns the furthest vertex of the polygon in the given direction.
func (p *Polygon) getSupport(dir Point) int {
	index := 0
	maxDist := Dot(dir, p.Points[index])
	for i := 1; i < len(p.Points); i++ {
		dist := Dot(dir, p.Points[i])
		if dist > maxDist {
			index = i
			maxDist = dist
		}
	}
	return index
}

func (p *Polygon) getVertex(index int) Point {
	return p.Points[index]
}

// Rectangle returns a rectangular polygon shape with the given center and half extents.
func Rectangle(center, extents Point) *Polygon {
	return NewPolygon(
		center.Add(Point{-extents.X, -extents.Y}),
		center.Add(Point{extents.X, -extents.Y}),
		center.Add(Point{extents.X, extents.Y}),
		center.Add(Point{-extents.X, extents.Y}),
	)
}

// Rect is shorthand for Rectangle(Point{x, y}, Point{w/2, h/2}).
func Rect(x, y, w, h float64) *Polygon {
	return Rectangle(Point{x, y}, Point{w / 2, h / 2})
}
