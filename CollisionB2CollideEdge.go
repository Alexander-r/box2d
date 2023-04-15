package box2d

import (
	"math"
)

// Compute contact points for edge versus circle.
// This accounts for edge connectivity.
func B2CollideEdgeAndCircle(manifold *B2Manifold, edgeA *B2EdgeShape, xfA B2Transform, circleB *B2CircleShape, xfB B2Transform) {
	manifold.PointCount = 0

	// Compute circle in frame of edge
	Q := B2TransformVec2MulT(xfA, B2TransformVec2Mul(xfB, circleB.M_p))

	A := edgeA.M_vertex1
	B := edgeA.M_vertex2
	e := B2Vec2Sub(B, A)

	// Normal points to the right for a CCW winding
	n := MakeB2Vec2(e.Y, -e.X)
	offset := B2Vec2Dot(n, B2Vec2Sub(Q, A))

	oneSided := edgeA.M_oneSided
	if oneSided && offset < 0.0 {
		return
	}

	// Barycentric coordinates
	u := B2Vec2Dot(e, B2Vec2Sub(B, Q))
	v := B2Vec2Dot(e, B2Vec2Sub(Q, A))

	radius := edgeA.M_radius + circleB.M_radius

	cf := MakeB2ContactFeature()
	cf.IndexB = 0
	cf.TypeB = B2ContactFeature_Type.E_vertex

	// Region A
	if v <= 0.0 {
		P := A
		d := B2Vec2Sub(Q, P)
		dd := B2Vec2Dot(d, d)
		if dd > radius*radius {
			return
		}

		// Is there an edge connected to A?
		if edgeA.M_oneSided {
			A1 := edgeA.M_vertex0
			B1 := A
			e1 := B2Vec2Sub(B1, A1)
			u1 := B2Vec2Dot(e1, B2Vec2Sub(B1, Q))

			// Is the circle in Region AB of the previous edge?
			if u1 > 0.0 {
				return
			}
		}

		cf.IndexA = 0
		cf.TypeA = B2ContactFeature_Type.E_vertex
		manifold.PointCount = 1
		manifold.Type = B2Manifold_Type.E_circles
		manifold.LocalNormal.SetZero()
		manifold.LocalPoint = P
		manifold.Points[0].Id.SetKey(0)
		manifold.Points[0].Id.IndexA = cf.IndexA
		manifold.Points[0].Id.IndexB = cf.IndexB
		manifold.Points[0].Id.TypeA = cf.TypeA
		manifold.Points[0].Id.TypeB = cf.TypeB
		manifold.Points[0].LocalPoint = circleB.M_p
		return
	}

	// Region B
	if u <= 0.0 {
		P := B
		d := B2Vec2Sub(Q, P)
		dd := B2Vec2Dot(d, d)
		if dd > radius*radius {
			return
		}

		// Is there an edge connected to B?
		if edgeA.M_oneSided {
			B2 := edgeA.M_vertex3
			A2 := B
			e2 := B2Vec2Sub(B2, A2)
			v2 := B2Vec2Dot(e2, B2Vec2Sub(Q, A2))

			// Is the circle in Region AB of the next edge?
			if v2 > 0.0 {
				return
			}
		}

		cf.IndexA = 1
		cf.TypeA = B2ContactFeature_Type.E_vertex
		manifold.PointCount = 1
		manifold.Type = B2Manifold_Type.E_circles
		manifold.LocalNormal.SetZero()
		manifold.LocalPoint = P
		manifold.Points[0].Id.SetKey(0)
		manifold.Points[0].Id.IndexA = cf.IndexA
		manifold.Points[0].Id.IndexB = cf.IndexB
		manifold.Points[0].Id.TypeA = cf.TypeA
		manifold.Points[0].Id.TypeB = cf.TypeB
		manifold.Points[0].LocalPoint = circleB.M_p
		return
	}

	// Region AB
	den := B2Vec2Dot(e, e)
	B2Assert(den > 0.0)
	P := B2Vec2MulScalar(1.0/den, B2Vec2Add(B2Vec2MulScalar(u, A), B2Vec2MulScalar(v, B)))
	d := B2Vec2Sub(Q, P)
	dd := B2Vec2Dot(d, d)
	if dd > radius*radius {
		return
	}

	if offset < 0.0 {
		n.Set(-n.X, -n.Y)
	}
	n.Normalize()

	cf.IndexA = 0
	cf.TypeA = B2ContactFeature_Type.E_face
	manifold.PointCount = 1
	manifold.Type = B2Manifold_Type.E_faceA
	manifold.LocalNormal = n
	manifold.LocalPoint = A
	manifold.Points[0].Id.SetKey(0)
	manifold.Points[0].Id.IndexA = cf.IndexA
	manifold.Points[0].Id.IndexB = cf.IndexB
	manifold.Points[0].Id.TypeA = cf.TypeA
	manifold.Points[0].Id.TypeB = cf.TypeB
	manifold.Points[0].LocalPoint = circleB.M_p
}

// This structure is used to keep track of the best separating axis.
var B2EPAxis_Type = struct {
	E_unknown uint8
	E_edgeA   uint8
	E_edgeB   uint8
}{
	E_unknown: 0,
	E_edgeA:   1,
	E_edgeB:   2,
}

type B2EPAxis struct {
	Normal     B2Vec2
	Type       uint8
	Index      int
	Separation float64
}

func MakeB2EPAxis() B2EPAxis {
	return B2EPAxis{}
}

// This holds polygon B expressed in frame A.
type B2TempPolygon struct {
	Vertices [B2_maxPolygonVertices]B2Vec2
	Normals  [B2_maxPolygonVertices]B2Vec2
	Count    int
}

// Reference face used for clipping
type B2ReferenceFace struct {
	I1, I2 int
	V1, V2 B2Vec2
	Normal B2Vec2

	SideNormal1 B2Vec2
	SideOffset1 float64

	SideNormal2 B2Vec2
	SideOffset2 float64
}

func MakeB2ReferenceFace() B2ReferenceFace {
	return B2ReferenceFace{}
}

func B2ComputeEdgeSeparation(polygonB B2TempPolygon, v1 B2Vec2, normal1 B2Vec2) B2EPAxis {
	axis := MakeB2EPAxis()
	axis.Type = B2EPAxis_Type.E_edgeA
	axis.Index = -1
	axis.Separation = -B2_maxFloat
	axis.Normal.SetZero()

	var axes [2]B2Vec2 = [2]B2Vec2{normal1, normal1.OperatorNegate()}

	// Find axis with least overlap (min-max problem)
	for j := 0; j < 2; j++ {
		sj := B2_maxFloat

		// Find deepest polygon vertex along axis j
		for i := 0; i < polygonB.Count; i++ {
			si := B2Vec2Dot(axes[j], B2Vec2Sub(polygonB.Vertices[i], v1))
			if si < sj {
				sj = si
			}
		}

		if sj > axis.Separation {
			axis.Index = j
			axis.Separation = sj
			axis.Normal = axes[j]
		}
	}

	return axis
}

func B2ComputePolygonSeparation(polygonB B2TempPolygon, v1 B2Vec2, v2 B2Vec2) B2EPAxis {
	axis := MakeB2EPAxis()
	axis.Type = B2EPAxis_Type.E_unknown
	axis.Index = -1
	axis.Separation = -B2_maxFloat
	axis.Normal.SetZero()

	for i := 0; i < polygonB.Count; i++ {
		n := polygonB.Normals[i].OperatorNegate()

		s1 := B2Vec2Dot(n, B2Vec2Sub(polygonB.Vertices[i], v1))
		s2 := B2Vec2Dot(n, B2Vec2Sub(polygonB.Vertices[i], v2))
		s := math.Min(s1, s2)

		if s > axis.Separation {
			axis.Type = B2EPAxis_Type.E_edgeB
			axis.Index = i
			axis.Separation = s
			axis.Normal = n
		}
	}

	return axis
}

// Compute the collision manifold between an edge and a polygon.
func B2CollideEdgeAndPolygon(manifold *B2Manifold, edgeA *B2EdgeShape, xfA B2Transform, polygonB *B2PolygonShape, xfB B2Transform) {
	manifold.PointCount = 0

	xf := B2TransformMulT(xfA, xfB)

	centroidB := B2TransformVec2Mul(xf, polygonB.M_centroid)

	v1 := edgeA.M_vertex1
	v2 := edgeA.M_vertex2

	edge1 := B2Vec2Sub(v2, v1)
	edge1.Normalize()

	// Normal points to the right for a CCW winding
	normal1 := MakeB2Vec2(edge1.Y, -edge1.X)
	offset1 := B2Vec2Dot(normal1, B2Vec2Sub(centroidB, v1))

	oneSided := edgeA.M_oneSided
	if oneSided && offset1 < 0.0 {
		return
	}

	// Get polygonB in frameA
	var tempPolygonB B2TempPolygon
	tempPolygonB.Count = polygonB.M_count
	for i := 0; i < polygonB.M_count; i++ {
		tempPolygonB.Vertices[i] = B2TransformVec2Mul(xf, polygonB.M_vertices[i])
		tempPolygonB.Normals[i] = B2RotVec2Mul(xf.Q, polygonB.M_normals[i])
	}

	radius := polygonB.M_radius + edgeA.M_radius

	edgeAxis := B2ComputeEdgeSeparation(tempPolygonB, v1, normal1)
	if edgeAxis.Separation > radius {
		return
	}

	polygonAxis := B2ComputePolygonSeparation(tempPolygonB, v1, v2)
	if polygonAxis.Separation > radius {
		return
	}

	// Use hysteresis for jitter reduction.
	k_relativeTol := 0.98
	k_absoluteTol := 0.001

	primaryAxis := MakeB2EPAxis()
	if polygonAxis.Separation-radius > k_relativeTol*(edgeAxis.Separation-radius)+k_absoluteTol {
		primaryAxis = polygonAxis
	} else {
		primaryAxis = edgeAxis
	}

	if oneSided {
		// Smooth collision
		// See https://box2d.org/posts/2020/06/ghost-collisions/

		edge0 := B2Vec2Sub(v1, edgeA.M_vertex0)
		edge0.Normalize()
		normal0 := MakeB2Vec2(edge0.Y, -edge0.X)
		convex1 := B2Vec2Cross(edge0, edge1) >= 0.0

		edge2 := B2Vec2Sub(edgeA.M_vertex3, v2)
		edge2.Normalize()
		normal2 := MakeB2Vec2(edge2.Y, -edge2.X)
		convex2 := B2Vec2Cross(edge1, edge2) >= 0.0

		sinTol := 0.1
		side1 := B2Vec2Dot(primaryAxis.Normal, edge1) <= 0.0

		// Check Gauss Map
		if side1 {
			if convex1 {
				if B2Vec2Cross(primaryAxis.Normal, normal0) > sinTol {
					// Skip region
					return
				}

				// Admit region
			} else {
				// Snap region
				primaryAxis = edgeAxis
			}
		} else {
			if convex2 {
				if B2Vec2Cross(normal2, primaryAxis.Normal) > sinTol {
					// Skip region
					return
				}

				// Admit region
			} else {
				// Snap region
				primaryAxis = edgeAxis
			}
		}
	}

	clipPoints := make([]B2ClipVertex, 2)
	ref := MakeB2ReferenceFace()
	if primaryAxis.Type == B2EPAxis_Type.E_edgeA {
		manifold.Type = B2Manifold_Type.E_faceA

		// Search for the polygon normal that is most anti-parallel to the edge normal.
		bestIndex := 0
		bestValue := B2Vec2Dot(primaryAxis.Normal, tempPolygonB.Normals[0])
		for i := 1; i < tempPolygonB.Count; i++ {
			value := B2Vec2Dot(primaryAxis.Normal, tempPolygonB.Normals[i])
			if value < bestValue {
				bestValue = value
				bestIndex = i
			}
		}

		i1 := bestIndex
		i2 := 0
		if i1+1 < tempPolygonB.Count {
			i2 = i1 + 1
		}

		clipPoints[0].V = tempPolygonB.Vertices[i1]
		clipPoints[0].Id.IndexA = 0
		clipPoints[0].Id.IndexB = uint8(i1)
		clipPoints[0].Id.TypeA = B2ContactFeature_Type.E_face
		clipPoints[0].Id.TypeB = B2ContactFeature_Type.E_vertex

		clipPoints[1].V = tempPolygonB.Vertices[i2]
		clipPoints[1].Id.IndexA = 0
		clipPoints[1].Id.IndexB = uint8(i2)
		clipPoints[1].Id.TypeA = B2ContactFeature_Type.E_face
		clipPoints[1].Id.TypeB = B2ContactFeature_Type.E_vertex

		ref.I1 = 0
		ref.I2 = 1
		ref.V1 = v1
		ref.V2 = v2
		ref.Normal = primaryAxis.Normal
		ref.SideNormal1 = edge1.OperatorNegate()
		ref.SideNormal2 = edge1
	} else {
		manifold.Type = B2Manifold_Type.E_faceB

		clipPoints[0].V = v2
		clipPoints[0].Id.IndexA = 1
		clipPoints[0].Id.IndexB = uint8(primaryAxis.Index)
		clipPoints[0].Id.TypeA = B2ContactFeature_Type.E_vertex
		clipPoints[0].Id.TypeB = B2ContactFeature_Type.E_face

		clipPoints[1].V = v1
		clipPoints[1].Id.IndexA = 0
		clipPoints[1].Id.IndexB = uint8(primaryAxis.Index)
		clipPoints[1].Id.TypeA = B2ContactFeature_Type.E_vertex
		clipPoints[1].Id.TypeB = B2ContactFeature_Type.E_face

		ref.I1 = primaryAxis.Index
		ref.I2 = 0
		if ref.I1+1 < tempPolygonB.Count {
			ref.I2 = ref.I1 + 1
		}
		ref.V1 = tempPolygonB.Vertices[ref.I1]
		ref.V2 = tempPolygonB.Vertices[ref.I2]
		ref.Normal = tempPolygonB.Normals[ref.I1]

		// CCW winding
		ref.SideNormal1.Set(ref.Normal.Y, -ref.Normal.X)
		ref.SideNormal2 = ref.SideNormal1.OperatorNegate()
	}

	ref.SideOffset1 = B2Vec2Dot(ref.SideNormal1, ref.V1)
	ref.SideOffset2 = B2Vec2Dot(ref.SideNormal2, ref.V2)

	// Clip incident edge against reference face side planes
	clipPoints1 := make([]B2ClipVertex, 2)
	clipPoints2 := make([]B2ClipVertex, 2)
	np := 0

	// Clip to side 1
	np = B2ClipSegmentToLine(clipPoints1, clipPoints, ref.SideNormal1, ref.SideOffset1, ref.I1)

	if np < B2_maxManifoldPoints {
		return
	}

	// Clip to side 2
	np = B2ClipSegmentToLine(clipPoints2, clipPoints1, ref.SideNormal2, ref.SideOffset2, ref.I2)

	if np < B2_maxManifoldPoints {
		return
	}

	// Now clipPoints2 contains the clipped points.
	if primaryAxis.Type == B2EPAxis_Type.E_edgeA {
		manifold.LocalNormal = ref.Normal
		manifold.LocalPoint = ref.V1
	} else {
		manifold.LocalNormal = polygonB.M_normals[ref.I1]
		manifold.LocalPoint = polygonB.M_vertices[ref.I1]
	}

	pointCount := 0
	for i := 0; i < B2_maxManifoldPoints; i++ {
		separation := 0.0

		separation = B2Vec2Dot(ref.Normal, B2Vec2Sub(clipPoints2[i].V, ref.V1))

		if separation <= radius {
			cp := &manifold.Points[pointCount]

			if primaryAxis.Type == B2EPAxis_Type.E_edgeA {
				cp.LocalPoint = B2TransformVec2MulT(xf, clipPoints2[i].V)
				cp.Id = clipPoints2[i].Id
			} else {
				cp.LocalPoint = clipPoints2[i].V
				cp.Id.TypeA = clipPoints2[i].Id.TypeB
				cp.Id.TypeB = clipPoints2[i].Id.TypeA
				cp.Id.IndexA = clipPoints2[i].Id.IndexB
				cp.Id.IndexB = clipPoints2[i].Id.IndexA
			}

			pointCount++
		}
	}

	manifold.PointCount = pointCount
}
