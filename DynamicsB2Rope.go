package box2d

import (
	"math"
)

var B2StretchingModel = struct {
	B2_pbdStretchingModel  uint8
	B2_xpbdStretchingModel uint8
}{
	B2_pbdStretchingModel:  1,
	B2_xpbdStretchingModel: 2,
}

var B2BendingModel = struct {
	B2_springAngleBendingModel uint8
	B2_pbdAngleBendingModel    uint8
	B2_xpbdAngleBendingModel   uint8
	B2_pbdDistanceBendingModel uint8
	B2_pbdHeightBendingModel   uint8
}{
	B2_springAngleBendingModel: 1,
	B2_pbdAngleBendingModel:    2,
	B2_xpbdAngleBendingModel:   3,
	B2_pbdDistanceBendingModel: 4,
	B2_pbdHeightBendingModel:   5,
}

///
type B2RopeTuning struct {
	StretchingModel    uint8
	BendingModel       uint8
	Damping            float64
	StretchStiffness   float64
	StretchHertz       float64
	StretchDamping     float64
	BendStiffness      float64
	BendHertz          float64
	BendDamping        float64
	Isometric          bool
	FixedEffectiveMass bool
	WarmStart          bool
}

func MakeB2RopeTuning() B2RopeTuning {
	res := B2RopeTuning{}

	res.StretchingModel = B2StretchingModel.B2_pbdStretchingModel
	res.BendingModel = B2BendingModel.B2_pbdAngleBendingModel
	res.Damping = 0.0
	res.StretchStiffness = 1.0
	res.BendStiffness = 0.5
	res.BendHertz = 1.0
	res.BendDamping = 0.0
	res.Isometric = false
	res.FixedEffectiveMass = false
	res.WarmStart = false

	return res
}

///
type B2RopeDef struct {
	Position B2Vec2
	Vertices []B2Vec2
	Count    int
	Masses   []float64
	Gravity  B2Vec2
	Tuning   B2RopeTuning
}

func MakeB2RopeDef() B2RopeDef {
	res := B2RopeDef{}

	res.Position.SetZero()
	res.Vertices = nil
	res.Count = 0
	res.Masses = nil
	res.Gravity.SetZero()
	res.Tuning = MakeB2RopeTuning()

	return res
}

type B2RopeStretch struct {
	I1       int
	I2       int
	InvMass1 float64
	InvMass2 float64
	L        float64
	Lambda   float64
	Spring   float64
	Damper   float64
}

type B2RopeBend struct {
	I1               int
	I2               int
	I3               int
	InvMass1         float64
	InvMass2         float64
	InvMass3         float64
	InvEffectiveMass float64
	Lambda           float64
	L1               float64
	L2               float64
	alpha1           float64
	alpha2           float64
	Spring           float64
	Damper           float64
}

///
type B2Rope struct {
	M_position B2Vec2

	M_count        int
	M_stretchCount int
	M_bendCount    int

	M_stretchConstraints []B2RopeStretch
	M_bendConstraints    []B2RopeBend

	M_bindPositions []B2Vec2
	M_ps            []B2Vec2
	M_p0s           []B2Vec2
	M_vs            []B2Vec2

	M_invMasses []float64
	M_gravity   B2Vec2

	M_tuning B2RopeTuning
}

///
func (rope B2Rope) GetVertexCount() int {
	return rope.M_count
}

///
func (rope B2Rope) GetVertices() []B2Vec2 {
	return rope.M_ps
}

func MakeB2Rope() B2Rope {
	res := B2Rope{}

	res.M_position.SetZero()
	res.M_count = 0
	res.M_stretchCount = 0
	res.M_bendCount = 0
	res.M_stretchConstraints = nil
	res.M_bendConstraints = nil
	res.M_bindPositions = nil
	res.M_ps = nil
	res.M_p0s = nil
	res.M_vs = nil
	res.M_invMasses = nil
	res.M_gravity.SetZero()
	res.M_tuning = MakeB2RopeTuning()

	return res
}

func (rope *B2Rope) Destroy() {
	rope.M_stretchConstraints = nil
	rope.M_bendConstraints = nil
	rope.M_bindPositions = nil
	rope.M_ps = nil
	rope.M_p0s = nil
	rope.M_vs = nil
	rope.M_invMasses = nil
}

func (rope *B2Rope) Create(def *B2RopeDef) {
	B2Assert(def.Count >= 3)
	rope.M_position = def.Position
	rope.M_count = def.Count
	rope.M_bindPositions = make([]B2Vec2, rope.M_count)
	rope.M_ps = make([]B2Vec2, rope.M_count)
	rope.M_p0s = make([]B2Vec2, rope.M_count)
	rope.M_vs = make([]B2Vec2, rope.M_count)
	rope.M_invMasses = make([]float64, rope.M_count)

	for i := 0; i < rope.M_count; i++ {
		rope.M_bindPositions[i] = def.Vertices[i]
		rope.M_ps[i] = B2Vec2Add(def.Vertices[i], rope.M_position)
		rope.M_p0s[i] = B2Vec2Add(def.Vertices[i], rope.M_position)
		rope.M_vs[i].SetZero()

		m := def.Masses[i]
		if m > 0.0 {
			rope.M_invMasses[i] = 1.0 / m
		} else {
			rope.M_invMasses[i] = 0.0
		}
	}

	rope.M_stretchCount = rope.M_count - 1
	rope.M_bendCount = rope.M_count - 2

	rope.M_stretchConstraints = make([]B2RopeStretch, rope.M_stretchCount)
	rope.M_bendConstraints = make([]B2RopeBend, rope.M_bendCount)

	for i := 0; i < rope.M_stretchCount; i++ {
		c := &rope.M_stretchConstraints[i]

		p1 := rope.M_ps[i]
		p2 := rope.M_ps[i+1]

		c.I1 = i
		c.I2 = i + 1
		c.L = B2Vec2Distance(p1, p2)
		c.InvMass1 = rope.M_invMasses[i]
		c.InvMass2 = rope.M_invMasses[i+1]
		c.Lambda = 0.0
		c.Damper = 0.0
		c.Spring = 0.0
	}

	for i := 0; i < rope.M_bendCount; i++ {
		c := &rope.M_bendConstraints[i]

		p1 := rope.M_ps[i]
		p2 := rope.M_ps[i+1]
		p3 := rope.M_ps[i+2]

		c.I1 = i
		c.I2 = i + 1
		c.I3 = i + 2
		c.InvMass1 = rope.M_invMasses[i]
		c.InvMass2 = rope.M_invMasses[i+1]
		rope.M_bendConstraints[i].InvMass3 = rope.M_invMasses[i+2]
		rope.M_bendConstraints[i].InvEffectiveMass = 0.0
		rope.M_bendConstraints[i].L1 = B2Vec2Distance(p1, p2)
		rope.M_bendConstraints[i].L2 = B2Vec2Distance(p2, p3)
		rope.M_bendConstraints[i].Lambda = 0.0

		// Pre-compute effective mass (TODO use flattened config)
		e1 := B2Vec2Sub(p2, p1)
		e2 := B2Vec2Sub(p3, p2)
		L1sqr := e1.LengthSquared()
		L2sqr := e2.LengthSquared()

		if L1sqr*L2sqr == 0.0 {
			continue
		}

		Jd1 := B2Vec2MulScalar((-1.0 / L1sqr), e1.Skew())
		Jd2 := B2Vec2MulScalar((1.0 / L2sqr), e2.Skew())

		J1 := Jd1.OperatorNegate()
		J2 := B2Vec2Sub(Jd1, Jd2)
		J3 := Jd2

		c.InvEffectiveMass = c.InvMass1*B2Vec2Dot(J1, J1) + c.InvMass2*B2Vec2Dot(J2, J2) + c.InvMass3*B2Vec2Dot(J3, J3)

		r := B2Vec2Sub(p3, p1)

		rr := r.LengthSquared()
		if rr == 0.0 {
			continue
		}

		// a1 = h2 / (h1 + h2)
		// a2 = h1 / (h1 + h2)
		c.alpha1 = B2Vec2Dot(e2, r) / rr
		c.alpha2 = B2Vec2Dot(e1, r) / rr
	}

	rope.M_gravity = def.Gravity

	rope.SetTuning(def.Tuning)
}

func (rope *B2Rope) SetTuning(tuning B2RopeTuning) {
	rope.M_tuning = tuning

	// Pre-compute spring and damper values based on tuning

	bendOmega := 2.0 * B2_pi * rope.M_tuning.BendHertz

	for i := 0; i < rope.M_bendCount; i++ {
		c := &rope.M_bendConstraints[i]

		L1sqr := c.L1 * c.L1
		L2sqr := c.L2 * c.L2

		if L1sqr*L2sqr == 0.0 {
			c.Spring = 0.0
			c.Damper = 0.0
			continue
		}

		// Flatten the triangle formed by the two edges
		J2 := 1.0/c.L1 + 1.0/c.L2
		sum := c.InvMass1/L1sqr + c.InvMass2*J2*J2 + c.InvMass3/L2sqr
		if sum == 0.0 {
			c.Spring = 0.0
			c.Damper = 0.0
			continue
		}

		mass := 1.0 / sum

		c.Spring = mass * bendOmega * bendOmega
		c.Damper = 2.0 * mass * rope.M_tuning.BendDamping * bendOmega
	}

	stretchOmega := 2.0 * B2_pi * rope.M_tuning.StretchHertz

	for i := 0; i < rope.M_stretchCount; i++ {
		c := &rope.M_stretchConstraints[i]

		sum := c.InvMass1 + c.InvMass2
		if sum == 0.0 {
			continue
		}

		mass := 1.0 / sum

		c.Spring = mass * stretchOmega * stretchOmega
		c.Damper = 2.0 * mass * rope.M_tuning.StretchDamping * stretchOmega
	}
}

func (rope *B2Rope) Step(dt float64, iterations int, position B2Vec2) {
	if dt == 0.0 {
		return
	}

	inv_dt := 1.0 / dt
	d := math.Exp(-dt * rope.M_tuning.Damping)

	// Apply gravity and damping
	for i := 0; i < rope.M_count; i++ {
		if rope.M_invMasses[i] > 0.0 {
			rope.M_vs[i].OperatorScalarMulInplace(d)
			rope.M_vs[i].OperatorPlusInplace(B2Vec2MulScalar(dt, rope.M_gravity))
		} else {
			rope.M_vs[i] = B2Vec2MulScalar(inv_dt, B2Vec2Sub(B2Vec2Add(rope.M_bindPositions[i], position), rope.M_p0s[i]))
		}
	}

	// Apply bending spring
	if rope.M_tuning.BendingModel == B2BendingModel.B2_springAngleBendingModel {
		rope.ApplyBendForces(dt)
	}

	for i := 0; i < rope.M_bendCount; i++ {
		rope.M_bendConstraints[i].Lambda = 0.0
	}

	for i := 0; i < rope.M_stretchCount; i++ {
		rope.M_stretchConstraints[i].Lambda = 0.0
	}

	// Update position
	for i := 0; i < rope.M_count; i++ {
		rope.M_ps[i].OperatorPlusInplace(B2Vec2MulScalar(dt, rope.M_vs[i]))
	}

	// Solve constraints
	for i := 0; i < iterations; i++ {
		if rope.M_tuning.BendingModel == B2BendingModel.B2_pbdAngleBendingModel {
			rope.SolveBend_PBD_Angle()
		} else if rope.M_tuning.BendingModel == B2BendingModel.B2_xpbdAngleBendingModel {
			rope.SolveBend_XPBD_Angle(dt)
		} else if rope.M_tuning.BendingModel == B2BendingModel.B2_pbdDistanceBendingModel {
			rope.SolveBend_PBD_Distance()
		} else if rope.M_tuning.BendingModel == B2BendingModel.B2_pbdHeightBendingModel {
			rope.SolveBend_PBD_Height()
		}

		if rope.M_tuning.StretchingModel == B2StretchingModel.B2_pbdStretchingModel {
			rope.SolveStretch_PBD()
		} else if rope.M_tuning.StretchingModel == B2StretchingModel.B2_xpbdStretchingModel {
			rope.SolveStretch_XPBD(dt)
		}
	}

	// Constrain velocity
	for i := 0; i < rope.M_count; i++ {
		rope.M_vs[i] = B2Vec2MulScalar(inv_dt, B2Vec2Sub(rope.M_ps[i], rope.M_p0s[i]))
		rope.M_p0s[i] = rope.M_ps[i]
	}
}

func (rope *B2Rope) Reset(position B2Vec2) {
	rope.M_position = position

	for i := 0; i < rope.M_count; i++ {
		rope.M_ps[i] = B2Vec2Add(rope.M_bindPositions[i], rope.M_position)
		rope.M_p0s[i] = B2Vec2Add(rope.M_bindPositions[i], rope.M_position)
		rope.M_vs[i].SetZero()
	}

	for i := 0; i < rope.M_bendCount; i++ {
		rope.M_bendConstraints[i].Lambda = 0.0
	}

	for i := 0; i < rope.M_stretchCount; i++ {
		rope.M_stretchConstraints[i].Lambda = 0.0
	}
}

func (rope *B2Rope) SolveStretch_PBD() {
	stiffness := rope.M_tuning.StretchStiffness

	for i := 0; i < rope.M_stretchCount; i++ {
		c := &rope.M_stretchConstraints[i]

		p1 := rope.M_ps[c.I1]
		p2 := rope.M_ps[c.I2]

		d := B2Vec2Sub(p2, p1)
		L := d.Normalize()

		sum := c.InvMass1 + c.InvMass2
		if sum == 0.0 {
			continue
		}

		s1 := c.InvMass1 / sum
		s2 := c.InvMass2 / sum

		p1.OperatorMinusInplace(B2Vec2MulScalar(stiffness*s1*(c.L-L), d))
		p2.OperatorPlusInplace(B2Vec2MulScalar(stiffness*s2*(c.L-L), d))

		rope.M_ps[c.I1] = p1
		rope.M_ps[c.I2] = p2
	}
}

func (rope *B2Rope) SolveStretch_XPBD(dt float64) {
	B2Assert(dt > 0.0)

	for i := 0; i < rope.M_stretchCount; i++ {
		c := &rope.M_stretchConstraints[i]

		p1 := rope.M_ps[c.I1]
		p2 := rope.M_ps[c.I2]

		dp1 := B2Vec2Sub(p1, rope.M_p0s[c.I1])
		dp2 := B2Vec2Sub(p2, rope.M_p0s[c.I2])

		u := B2Vec2Sub(p2, p1)
		L := u.Normalize()

		J1 := u.OperatorNegate()
		J2 := u

		sum := c.InvMass1 + c.InvMass2
		if sum == 0.0 {
			continue
		}

		alpha := 1.0 / (c.Spring * dt * dt) // 1 / kg
		beta := dt * dt * c.Damper          // kg * s
		sigma := alpha * beta / dt          // non-dimensional
		C := L - c.L

		// This is using the initial velocities
		Cdot := B2Vec2Dot(J1, dp1) + B2Vec2Dot(J2, dp2)

		B := C + alpha*c.Lambda + sigma*Cdot
		sum2 := (1.0+sigma)*sum + alpha

		impulse := -B / sum2

		p1.OperatorPlusInplace(B2Vec2MulScalar((c.InvMass1 * impulse), J1))
		p2.OperatorPlusInplace(B2Vec2MulScalar((c.InvMass2 * impulse), J2))

		rope.M_ps[c.I1] = p1
		rope.M_ps[c.I2] = p2
		c.Lambda += impulse
	}
}

func (rope *B2Rope) SolveBend_PBD_Angle() {
	stiffness := rope.M_tuning.BendStiffness

	for i := 0; i < rope.M_bendCount; i++ {
		c := &rope.M_bendConstraints[i]

		p1 := rope.M_ps[c.I1]
		p2 := rope.M_ps[c.I2]
		p3 := rope.M_ps[c.I3]

		d1 := B2Vec2Sub(p2, p1)
		d2 := B2Vec2Sub(p3, p2)
		a := B2Vec2Cross(d1, d2)
		b := B2Vec2Dot(d1, d2)

		angle := math.Atan2(a, b)

		var L1sqr float64
		var L2sqr float64

		if rope.M_tuning.Isometric {
			L1sqr = c.L1 * c.L1
			L2sqr = c.L2 * c.L2
		} else {
			L1sqr = d1.LengthSquared()
			L2sqr = d2.LengthSquared()
		}

		if L1sqr*L2sqr == 0.0 {
			continue
		}

		Jd1 := B2Vec2MulScalar((-1.0 / L1sqr), d1.Skew())
		Jd2 := B2Vec2MulScalar((1.0 / L2sqr), d2.Skew())

		J1 := Jd1.OperatorNegate()
		J2 := B2Vec2Sub(Jd1, Jd2)
		J3 := Jd2

		var sum float64
		if rope.M_tuning.FixedEffectiveMass {
			sum = c.InvEffectiveMass
		} else {
			sum = c.InvMass1*B2Vec2Dot(J1, J1) + c.InvMass2*B2Vec2Dot(J2, J2) + c.InvMass3*B2Vec2Dot(J3, J3)
		}

		if sum == 0.0 {
			sum = c.InvEffectiveMass
		}

		impulse := -stiffness * angle / sum

		p1.OperatorPlusInplace(B2Vec2MulScalar((c.InvMass1 * impulse), J1))
		p2.OperatorPlusInplace(B2Vec2MulScalar((c.InvMass2 * impulse), J2))
		p3.OperatorPlusInplace(B2Vec2MulScalar((c.InvMass3 * impulse), J3))

		rope.M_ps[c.I1] = p1
		rope.M_ps[c.I2] = p2
		rope.M_ps[c.I3] = p3
	}
}

func (rope *B2Rope) SolveBend_XPBD_Angle(dt float64) {
	B2Assert(dt > 0.0)

	for i := 0; i < rope.M_bendCount; i++ {
		c := &rope.M_bendConstraints[i]

		p1 := rope.M_ps[c.I1]
		p2 := rope.M_ps[c.I2]
		p3 := rope.M_ps[c.I3]

		dp1 := B2Vec2Sub(p1, rope.M_p0s[c.I1])
		dp2 := B2Vec2Sub(p2, rope.M_p0s[c.I2])
		dp3 := B2Vec2Sub(p3, rope.M_p0s[c.I3])

		d1 := B2Vec2Sub(p2, p1)
		d2 := B2Vec2Sub(p3, p2)

		var L1sqr float64
		var L2sqr float64

		if rope.M_tuning.Isometric {
			L1sqr = c.L1 * c.L1
			L2sqr = c.L2 * c.L2
		} else {
			L1sqr = d1.LengthSquared()
			L2sqr = d2.LengthSquared()
		}

		if L1sqr*L2sqr == 0.0 {
			continue
		}

		a := B2Vec2Cross(d1, d2)
		b := B2Vec2Dot(d1, d2)

		angle := math.Atan2(a, b)

		Jd1 := B2Vec2MulScalar((-1.0 / L1sqr), d1.Skew())
		Jd2 := B2Vec2MulScalar((1.0 / L2sqr), d2.Skew())

		J1 := Jd1.OperatorNegate()
		J2 := B2Vec2Sub(Jd1, Jd2)
		J3 := Jd2

		var sum float64
		if rope.M_tuning.FixedEffectiveMass {
			sum = c.InvEffectiveMass
		} else {
			sum = c.InvMass1*B2Vec2Dot(J1, J1) + c.InvMass2*B2Vec2Dot(J2, J2) + c.InvMass3*B2Vec2Dot(J3, J3)
		}

		if sum == 0.0 {
			continue
		}

		alpha := 1.0 / (c.Spring * dt * dt)
		beta := dt * dt * c.Damper
		sigma := alpha * beta / dt
		C := angle

		// This is using the initial velocities
		Cdot := B2Vec2Dot(J1, dp1) + B2Vec2Dot(J2, dp2) + B2Vec2Dot(J3, dp3)

		B := C + alpha*c.Lambda + sigma*Cdot
		sum2 := (1.0+sigma)*sum + alpha

		impulse := -B / sum2

		p1.OperatorPlusInplace(B2Vec2MulScalar((c.InvMass1 * impulse), J1))
		p2.OperatorPlusInplace(B2Vec2MulScalar((c.InvMass2 * impulse), J2))
		p3.OperatorPlusInplace(B2Vec2MulScalar((c.InvMass3 * impulse), J3))

		rope.M_ps[c.I1] = p1
		rope.M_ps[c.I2] = p2
		rope.M_ps[c.I3] = p3
		c.Lambda += impulse
	}
}

func (rope *B2Rope) ApplyBendForces(dt float64) {
	// omega = 2 * pi * hz
	omega := 2.0 * B2_pi * rope.M_tuning.BendHertz

	for i := 0; i < rope.M_bendCount; i++ {
		c := &rope.M_bendConstraints[i]

		p1 := rope.M_ps[c.I1]
		p2 := rope.M_ps[c.I2]
		p3 := rope.M_ps[c.I3]

		v1 := rope.M_vs[c.I1]
		v2 := rope.M_vs[c.I2]
		v3 := rope.M_vs[c.I3]

		d1 := B2Vec2Sub(p2, p1)
		d2 := B2Vec2Sub(p3, p2)

		var L1sqr float64
		var L2sqr float64

		if rope.M_tuning.Isometric {
			L1sqr = c.L1 * c.L1
			L2sqr = c.L2 * c.L2
		} else {
			L1sqr = d1.LengthSquared()
			L2sqr = d2.LengthSquared()
		}

		if L1sqr*L2sqr == 0.0 {
			continue
		}

		a := B2Vec2Cross(d1, d2)
		b := B2Vec2Dot(d1, d2)

		angle := math.Atan2(a, b)

		Jd1 := B2Vec2MulScalar((-1.0 / L1sqr), d1.Skew())
		Jd2 := B2Vec2MulScalar((1.0 / L2sqr), d2.Skew())

		J1 := Jd1.OperatorNegate()
		J2 := B2Vec2Sub(Jd1, Jd2)
		J3 := Jd2

		var sum float64
		if rope.M_tuning.FixedEffectiveMass {
			sum = c.InvEffectiveMass
		} else {
			sum = c.InvMass1*B2Vec2Dot(J1, J1) + c.InvMass2*B2Vec2Dot(J2, J2) + c.InvMass3*B2Vec2Dot(J3, J3)
		}

		if sum == 0.0 {
			continue
		}

		mass := 1.0 / sum

		spring := mass * omega * omega
		damper := 2.0 * mass * rope.M_tuning.BendDamping * omega

		C := angle
		Cdot := B2Vec2Dot(J1, v1) + B2Vec2Dot(J2, v2) + B2Vec2Dot(J3, v3)

		impulse := -dt * (spring*C + damper*Cdot)

		rope.M_vs[c.I1].OperatorPlusInplace(B2Vec2MulScalar((c.InvMass1 * impulse), J1))
		rope.M_vs[c.I2].OperatorPlusInplace(B2Vec2MulScalar((c.InvMass2 * impulse), J2))
		rope.M_vs[c.I3].OperatorPlusInplace(B2Vec2MulScalar((c.InvMass3 * impulse), J3))
	}
}

func (rope *B2Rope) SolveBend_PBD_Distance() {
	stiffness := rope.M_tuning.BendStiffness

	for i := 0; i < rope.M_bendCount; i++ {

		c := &rope.M_bendConstraints[i]

		i1 := c.I1
		i2 := c.I3

		p1 := rope.M_ps[i1]
		p2 := rope.M_ps[i2]

		d := B2Vec2Sub(p2, p1)
		L := d.Normalize()

		sum := c.InvMass1 + c.InvMass3
		if sum == 0.0 {
			continue
		}

		s1 := c.InvMass1 / sum
		s2 := c.InvMass3 / sum

		p1.OperatorMinusInplace(B2Vec2MulScalar(stiffness*s1*(c.L1+c.L2-L), d))
		p2.OperatorPlusInplace(B2Vec2MulScalar(stiffness*s2*(c.L1+c.L2-L), d))

		rope.M_ps[i1] = p1
		rope.M_ps[i2] = p2
	}
}

// Constraint based implementation of:
// P. Volino: Simple Linear Bending Stiffness in Particle Systems
func (rope *B2Rope) SolveBend_PBD_Height() {
	stiffness := rope.M_tuning.BendStiffness

	for i := 0; i < rope.M_bendCount; i++ {
		c := &rope.M_bendConstraints[i]

		p1 := rope.M_ps[c.I1]
		p2 := rope.M_ps[c.I2]
		p3 := rope.M_ps[c.I3]

		// Barycentric coordinates are held constant
		d := B2Vec2Sub(B2Vec2Add(B2Vec2MulScalar(c.alpha1, p1), B2Vec2MulScalar(c.alpha2, p3)), p2)
		dLen := d.Length()

		if dLen == 0.0 {
			continue
		}

		dHat := B2Vec2MulScalar((1.0 / dLen), d)

		J1 := B2Vec2MulScalar(c.alpha1, dHat)
		J2 := dHat.OperatorNegate()
		J3 := B2Vec2MulScalar(c.alpha2, dHat)

		sum := c.InvMass1*c.alpha1*c.alpha1 + c.InvMass2 + c.InvMass3*c.alpha2*c.alpha2

		if sum == 0.0 {
			continue
		}

		C := dLen
		mass := 1.0 / sum
		impulse := -stiffness * mass * C

		p1.OperatorPlusInplace(B2Vec2MulScalar((c.InvMass1 * impulse), J1))
		p2.OperatorPlusInplace(B2Vec2MulScalar((c.InvMass2 * impulse), J2))
		p3.OperatorPlusInplace(B2Vec2MulScalar((c.InvMass3 * impulse), J3))

		rope.M_ps[c.I1] = p1
		rope.M_ps[c.I2] = p2
		rope.M_ps[c.I3] = p3
	}
}

//void b2Rope::Draw(b2Draw* draw) const
//{
//	b2Color c(0.4f, 0.5f, 0.7f);
//	b2Color pg(0.1f, 0.8f, 0.1f);
//	b2Color pd(0.7f, 0.2f, 0.4f);
//
//	for (int32 i = 0; i < m_count - 1; ++i)
//	{
//		draw->DrawSegment(m_ps[i], m_ps[i+1], c);
//
//		const b2Color& pc = m_invMasses[i] > 0.0f ? pd : pg;
//		draw->DrawPoint(m_ps[i], 5.0f, pc);
//	}
//
//	const b2Color& pc = m_invMasses[m_count - 1] > 0.0f ? pd : pg;
//	draw->DrawPoint(m_ps[m_count - 1], 5.0f, pc);
//}
