package box2d

// A solid convex polygon. It is assumed that the interior of the polygon is to
// the left of each edge.
// Polygons have a maximum number of vertices equal to b2_maxPolygonVertices.
// In most cases you should not need many vertices for a convex polygon.

type B2PolygonShape struct {
	B2Shape

	M_centroid B2Vec2
	M_vertices [B2_maxPolygonVertices]B2Vec2
	M_normals  [B2_maxPolygonVertices]B2Vec2
	M_count    int
}

func MakeB2PolygonShape() B2PolygonShape {
	return B2PolygonShape{
		B2Shape: B2Shape{
			M_type:   B2Shape_Type.E_polygon,
			M_radius: B2_polygonRadius,
		},
		M_count:    0,
		M_centroid: MakeB2Vec2(0, 0),
	}
}

func NewB2PolygonShape() *B2PolygonShape {
	res := MakeB2PolygonShape()
	return &res
}

func (poly *B2PolygonShape) GetVertex(index int) *B2Vec2 {
	B2Assert(0 <= index && index < poly.M_count)
	return &poly.M_vertices[index]
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2PolygonShape.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func (poly B2PolygonShape) Clone() B2ShapeInterface {

	clone := NewB2PolygonShape()
	clone.M_centroid = poly.M_centroid
	clone.M_count = poly.M_count

	for i, _ := range poly.M_vertices {
		clone.M_vertices[i] = poly.M_vertices[i]
	}

	for i, _ := range poly.M_normals {
		clone.M_normals[i] = poly.M_normals[i]
	}

	return clone
}

func (edge *B2PolygonShape) Destroy() {}

func (poly *B2PolygonShape) SetAsBox(hx float64, hy float64) {
	poly.M_count = 4
	poly.M_vertices[0].Set(-hx, -hy)
	poly.M_vertices[1].Set(hx, -hy)
	poly.M_vertices[2].Set(hx, hy)
	poly.M_vertices[3].Set(-hx, hy)
	poly.M_normals[0].Set(0.0, -1.0)
	poly.M_normals[1].Set(1.0, 0.0)
	poly.M_normals[2].Set(0.0, 1.0)
	poly.M_normals[3].Set(-1.0, 0.0)
	poly.M_centroid.SetZero()
}

func (poly *B2PolygonShape) SetAsBoxFromCenterAndAngle(hx float64, hy float64, center B2Vec2, angle float64) {
	poly.M_count = 4
	poly.M_vertices[0].Set(-hx, -hy)
	poly.M_vertices[1].Set(hx, -hy)
	poly.M_vertices[2].Set(hx, hy)
	poly.M_vertices[3].Set(-hx, hy)
	poly.M_normals[0].Set(0.0, -1.0)
	poly.M_normals[1].Set(1.0, 0.0)
	poly.M_normals[2].Set(0.0, 1.0)
	poly.M_normals[3].Set(-1.0, 0.0)
	poly.M_centroid = center

	xf := MakeB2Transform()
	xf.P = center
	xf.Q.Set(angle)

	// Transform vertices and normals.
	for i := 0; i < poly.M_count; i++ {
		poly.M_vertices[i] = B2TransformVec2Mul(xf, poly.M_vertices[i])
		poly.M_normals[i] = B2RotVec2Mul(xf.Q, poly.M_normals[i])
	}
}

func (poly B2PolygonShape) GetChildCount() int {
	return 1
}

func ComputeCentroid(vs []B2Vec2, count int) B2Vec2 {

	B2Assert(count >= 3)

	c := MakeB2Vec2(0, 0)
	area := 0.0

	// Get a reference point for forming triangles.
	// Use the first vertex to reduce round-off errors.
	s := vs[0]

	inv3 := 1.0 / 3.0

	for i := 0; i < count; i++ {
		// Triangle vertices.
		p1 := B2Vec2Sub(vs[0], s)
		p2 := B2Vec2Sub(vs[i], s)
		p3 := MakeB2Vec2(0, 0)
		if i+1 < count {
			p3 = B2Vec2Sub(vs[i+1], s)
		} else {
			p3 = B2Vec2Sub(vs[0], s)
		}

		e1 := B2Vec2Sub(p2, p1)
		e2 := B2Vec2Sub(p3, p1)

		D := B2Vec2Cross(e1, e2)

		triangleArea := 0.5 * D
		area += triangleArea

		// Area weighted centroid
		c.OperatorPlusInplace(B2Vec2MulScalar(triangleArea*inv3, B2Vec2Add(B2Vec2Add(p1, p2), p3)))
	}

	// Centroid
	B2Assert(area > B2_epsilon)
	c = B2Vec2Add(B2Vec2MulScalar(1.0/area, c), s)
	return c
}

// Create a convex hull from the given array of local points.
// The count must be in the range [3, b2_maxPolygonVertices].
// @warning the points may be re-ordered, even if they form a convex polygon
// @warning collinear points are handled but not removed. Collinear points
// may lead to poor stacking behavior.
func (poly *B2PolygonShape) Set(vertices []B2Vec2, count int) bool {
	hull := B2ComputeHull(vertices, count)

	if hull.Count < 3 {
		return false
	}

	poly.SetAsHull(hull)

	return true
}

// Create a polygon from a given convex hull (see b2ComputeHull).
// @warning the hull must be valid or this will crash or have unexpected behavior
func (poly *B2PolygonShape) SetAsHull(hull B2Hull) {
	B2Assert(hull.Count >= 3)

	poly.M_count = hull.Count

	// Copy vertices
	for i := 0; i < hull.Count; i++ {
		poly.M_vertices[i] = hull.Points[i]
	}

	// Compute normals. Ensure the edges have non-zero length.
	for i := 0; i < poly.M_count; i++ {
		i1 := i
		var i2 int
		if i+1 < poly.M_count {
			i2 = i + 1
		} else {
			i2 = 0
		}
		edge := B2Vec2Sub(poly.M_vertices[i2], poly.M_vertices[i1])
		B2Assert(edge.LengthSquared() > B2_epsilon*B2_epsilon)
		poly.M_normals[i] = B2Vec2CrossVectorScalar(edge, 1.0)
		poly.M_normals[i].Normalize()
	}

	// Compute the polygon centroid.
	poly.M_centroid = ComputeCentroid(poly.M_vertices[:], poly.M_count)
}

func (poly B2PolygonShape) TestPoint(xf B2Transform, p B2Vec2) bool {
	pLocal := B2RotVec2MulT(xf.Q, B2Vec2Sub(p, xf.P))

	for i := 0; i < poly.M_count; i++ {
		dot := B2Vec2Dot(poly.M_normals[i], B2Vec2Sub(pLocal, poly.M_vertices[i]))
		if dot > 0.0 {
			return false
		}
	}

	return true
}

// @note because the polygon is solid, rays that start inside do not hit because the normal is
// not defined.
func (poly B2PolygonShape) RayCast(output *B2RayCastOutput, input B2RayCastInput, xf B2Transform, childIndex int) bool {

	// Put the ray into the polygon's frame of reference.
	p1 := B2RotVec2MulT(xf.Q, B2Vec2Sub(input.P1, xf.P))
	p2 := B2RotVec2MulT(xf.Q, B2Vec2Sub(input.P2, xf.P))
	d := B2Vec2Sub(p2, p1)

	lower := 0.0
	upper := input.MaxFraction

	index := -1

	for i := 0; i < poly.M_count; i++ {
		// p = p1 + a * d
		// dot(normal, p - v) = 0
		// dot(normal, p1 - v) + a * dot(normal, d) = 0
		numerator := B2Vec2Dot(poly.M_normals[i], B2Vec2Sub(poly.M_vertices[i], p1))
		denominator := B2Vec2Dot(poly.M_normals[i], d)

		if denominator == 0.0 {
			if numerator < 0.0 {
				return false
			}
		} else {
			// Note: we want this predicate without division:
			// lower < numerator / denominator, where denominator < 0
			// Since denominator < 0, we have to flip the inequality:
			// lower < numerator / denominator <==> denominator * lower > numerator.
			if denominator < 0.0 && numerator < lower*denominator {
				// Increase lower.
				// The segment enters this half-space.
				lower = numerator / denominator
				index = i
			} else if denominator > 0.0 && numerator < upper*denominator {
				// Decrease upper.
				// The segment exits this half-space.
				upper = numerator / denominator
			}
		}

		// The use of epsilon here causes the assert on lower to trip
		// in some cases. Apparently the use of epsilon was to make edge
		// shapes work, but now those are handled separately.
		//if (upper < lower - b2_epsilon)
		if upper < lower {
			return false
		}
	}

	B2Assert(0.0 <= lower && lower <= input.MaxFraction)

	if index >= 0 {
		output.Fraction = lower
		output.Normal = B2RotVec2Mul(xf.Q, poly.M_normals[index])
		return true
	}

	return false
}

func (poly B2PolygonShape) ComputeAABB(aabb *B2AABB, xf B2Transform, childIndex int) {

	lower := B2TransformVec2Mul(xf, poly.M_vertices[0])
	upper := lower

	for i := 1; i < poly.M_count; i++ {
		v := B2TransformVec2Mul(xf, poly.M_vertices[i])
		lower = B2Vec2Min(lower, v)
		upper = B2Vec2Max(upper, v)
	}

	r := MakeB2Vec2(poly.M_radius, poly.M_radius)
	aabb.LowerBound = B2Vec2Sub(lower, r)
	aabb.UpperBound = B2Vec2Sub(upper, r)
}

func (poly B2PolygonShape) ComputeMass(massData *B2MassData, density float64) {
	// Polygon mass, centroid, and inertia.
	// Let rho be the polygon density in mass per unit area.
	// Then:
	// mass = rho * int(dA)
	// centroid.x = (1/mass) * rho * int(x * dA)
	// centroid.y = (1/mass) * rho * int(y * dA)
	// I = rho * int((x*x + y*y) * dA)
	//
	// We can compute these integrals by summing all the integrals
	// for each triangle of the polygon. To evaluate the integral
	// for a single triangle, we make a change of variables to
	// the (u,v) coordinates of the triangle:
	// x = x0 + e1x * u + e2x * v
	// y = y0 + e1y * u + e2y * v
	// where 0 <= u && 0 <= v && u + v <= 1.
	//
	// We integrate u from [0,1-v] and then v from [0,1].
	// We also need to use the Jacobian of the transformation:
	// D = cross(e1, e2)
	//
	// Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
	//
	// The rest of the derivation is handled by computer algebra.

	B2Assert(poly.M_count >= 3)

	center := MakeB2Vec2(0, 0)

	area := 0.0
	I := 0.0

	// Get a reference point for forming triangles.
	// Use the first vertex to reduce round-off errors.
	s := poly.M_vertices[0]

	k_inv3 := 1.0 / 3.0

	for i := 0; i < poly.M_count; i++ {
		// Triangle vertices.
		e1 := B2Vec2Sub(poly.M_vertices[i], s)
		e2 := MakeB2Vec2(0, 0)

		if i+1 < poly.M_count {
			e2 = B2Vec2Sub(poly.M_vertices[i+1], s)
		} else {
			e2 = B2Vec2Sub(poly.M_vertices[0], s)
		}

		D := B2Vec2Cross(e1, e2)

		triangleArea := 0.5 * D
		area += triangleArea

		// Area weighted centroid
		center.OperatorPlusInplace(B2Vec2MulScalar(triangleArea*k_inv3, B2Vec2Add(e1, e2)))

		ex1 := e1.X
		ey1 := e1.Y
		ex2 := e2.X
		ey2 := e2.Y

		intx2 := ex1*ex1 + ex2*ex1 + ex2*ex2
		inty2 := ey1*ey1 + ey2*ey1 + ey2*ey2

		I += (0.25 * k_inv3 * D) * (intx2 + inty2)
	}

	// Total mass
	massData.Mass = density * area

	// Center of mass
	B2Assert(area > B2_epsilon)
	center.OperatorScalarMulInplace(1.0 / area)
	massData.Center = B2Vec2Add(center, s)

	// Inertia tensor relative to the local origin (point s).
	massData.I = density * I

	// Shift to center of mass then to original body origin.
	massData.I += massData.Mass * (B2Vec2Dot(massData.Center, massData.Center) - B2Vec2Dot(center, center))
}

func (poly B2PolygonShape) Validate() bool {
	if poly.M_count < 3 || B2_maxPolygonVertices < poly.M_count {
		return false
	}

	var hull B2Hull
	for i := 0; i < poly.M_count; i++ {
		hull.Points[i] = poly.M_vertices[i]
	}

	hull.Count = poly.M_count

	return B2ValidateHull(&hull)
}
