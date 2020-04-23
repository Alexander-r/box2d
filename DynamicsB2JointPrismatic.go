package box2d

import (
	"fmt"
	"math"
)

/// Prismatic joint definition. This requires defining a line of
/// motion using an axis and an anchor point. The definition uses local
/// anchor points and a local axis so that the initial configuration
/// can violate the constraint slightly. The joint translation is zero
/// when the local anchor points coincide in world space. Using local
/// anchors and a local axis helps when saving and loading a game.
type B2PrismaticJointDef struct {
	B2JointDef

	/// The local anchor point relative to bodyA's origin.
	LocalAnchorA B2Vec2

	/// The local anchor point relative to bodyB's origin.
	LocalAnchorB B2Vec2

	/// The local translation unit axis in bodyA.
	LocalAxisA B2Vec2

	/// The constrained angle between the bodies: bodyB_angle - bodyA_angle.
	ReferenceAngle float64

	/// Enable/disable the joint limit.
	EnableLimit bool

	/// The lower translation limit, usually in meters.
	LowerTranslation float64

	/// The upper translation limit, usually in meters.
	UpperTranslation float64

	/// Enable/disable the joint motor.
	EnableMotor bool

	/// The maximum motor torque, usually in N-m.
	MaxMotorForce float64

	/// The desired motor speed in radians per second.
	MotorSpeed float64
}

func MakeB2PrismaticJointDef() B2PrismaticJointDef {
	res := B2PrismaticJointDef{
		B2JointDef: MakeB2JointDef(),
	}

	res.Type = B2JointType.E_prismaticJoint
	res.LocalAnchorA.SetZero()
	res.LocalAnchorB.SetZero()
	res.LocalAxisA.Set(1.0, 0.0)
	res.ReferenceAngle = 0.0
	res.EnableLimit = false
	res.LowerTranslation = 0.0
	res.UpperTranslation = 0.0
	res.EnableMotor = false
	res.MaxMotorForce = 0.0
	res.MotorSpeed = 0.0

	return res
}

/// A prismatic joint. This joint provides one degree of freedom: translation
/// along an axis fixed in bodyA. Relative rotation is prevented. You can
/// use a joint limit to restrict the range of motion and a joint motor to
/// drive the motion or to model joint friction.
type B2PrismaticJoint struct {
	*B2Joint

	M_localAnchorA     B2Vec2
	M_localAnchorB     B2Vec2
	M_localXAxisA      B2Vec2
	M_localYAxisA      B2Vec2
	M_referenceAngle   float64
	M_impulse          B2Vec2
	M_motorImpulse     float64
	M_lowerImpulse     float64
	M_upperImpulse     float64
	M_lowerTranslation float64
	M_upperTranslation float64
	M_maxMotorForce    float64
	M_motorSpeed       float64
	M_enableLimit      bool
	M_enableMotor      bool

	// Solver temp
	M_indexA       int
	M_indexB       int
	M_localCenterA B2Vec2
	M_localCenterB B2Vec2
	M_invMassA     float64
	M_invMassB     float64
	M_invIA        float64
	M_invIB        float64
	M_axis, M_perp B2Vec2
	M_s1, M_s2     float64
	M_a1, M_a2     float64
	M_K            B2Mat22
	M_translation  float64
	M_axialMass    float64
}

/// The local anchor point relative to bodyA's origin.
func (joint B2PrismaticJoint) GetLocalAnchorA() B2Vec2 {
	return joint.M_localAnchorA
}

/// The local anchor point relative to bodyB's origin.
func (joint B2PrismaticJoint) GetLocalAnchorB() B2Vec2 {
	return joint.M_localAnchorB
}

/// The local joint axis relative to bodyA.
func (joint B2PrismaticJoint) GetLocalAxisA() B2Vec2 {
	return joint.M_localXAxisA
}

/// Get the reference angle.
func (joint B2PrismaticJoint) GetReferenceAngle() float64 {
	return joint.M_referenceAngle
}

func (joint B2PrismaticJoint) GetMaxMotorForce() float64 {
	return joint.M_maxMotorForce
}

func (joint B2PrismaticJoint) GetMotorSpeed() float64 {
	return joint.M_motorSpeed
}

// Linear constraint (point-to-line)
// d = p2 - p1 = x2 + r2 - x1 - r1
// C = dot(perp, d)
// Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
//      = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
// J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
//
// Angular constraint
// C = a2 - a1 + a_initial
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
//
// K = J * invM * JT
//
// J = [-a -s1 a s2]
//     [0  -1  0  1]
// a = perp
// s1 = cross(d + r1, a) = cross(p2 - x1, a)
// s2 = cross(r2, a) = cross(p2 - x2, a)

// Motor/Limit linear constraint
// C = dot(ax1, d)
// Cdot = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
// J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]

// Predictive limit is applied even when the limit is not active.
// Prevents a constraint speed that can lead to a constraint error in one time step.
// Want C2 = C1 + h * Cdot >= 0
// Or:
// Cdot + C1/h >= 0
// I do not apply a negative constraint error because that is handled in position correction.
// So:
// Cdot + max(C1, 0)/h >= 0

// Block Solver
// We develop a block solver that includes the angular and linear constraints. This makes the limit stiffer.
//
// The Jacobian has 2 rows:
// J = [-uT -s1 uT s2] // linear
//     [0   -1   0  1] // angular
//
// u = perp
// s1 = cross(d + r1, u), s2 = cross(r2, u)
// a1 = cross(d + r1, v), a2 = cross(r2, v)

func (joint *B2PrismaticJointDef) Initialize(bA *B2Body, bB *B2Body, anchor B2Vec2, axis B2Vec2) {
	joint.BodyA = bA
	joint.BodyB = bB
	joint.LocalAnchorA = joint.BodyA.GetLocalPoint(anchor)
	joint.LocalAnchorB = joint.BodyB.GetLocalPoint(anchor)
	joint.LocalAxisA = joint.BodyA.GetLocalVector(axis)
	joint.ReferenceAngle = joint.BodyB.GetAngle() - joint.BodyA.GetAngle()
}

func MakeB2PrismaticJoint(def *B2PrismaticJointDef) *B2PrismaticJoint {
	res := B2PrismaticJoint{
		B2Joint: MakeB2Joint(def),
	}

	res.M_localAnchorA = def.LocalAnchorA
	res.M_localAnchorB = def.LocalAnchorB
	res.M_localXAxisA = def.LocalAxisA
	res.M_localXAxisA.Normalize()
	res.M_localYAxisA = B2Vec2CrossScalarVector(1.0, res.M_localXAxisA)
	res.M_referenceAngle = def.ReferenceAngle

	res.M_impulse.SetZero()
	res.M_axialMass = 0.0
	res.M_motorImpulse = 0.0
	res.M_lowerImpulse = 0.0
	res.M_upperImpulse = 0.0

	res.M_lowerTranslation = def.LowerTranslation
	res.M_upperTranslation = def.UpperTranslation

	B2Assert(res.M_lowerTranslation <= res.M_upperTranslation)

	res.M_maxMotorForce = def.MaxMotorForce
	res.M_motorSpeed = def.MotorSpeed
	res.M_enableLimit = def.EnableLimit
	res.M_enableMotor = def.EnableMotor

	res.M_axis.SetZero()
	res.M_perp.SetZero()

	return &res
}

func (joint *B2PrismaticJoint) InitVelocityConstraints(data B2SolverData) {
	joint.M_indexA = joint.M_bodyA.M_islandIndex
	joint.M_indexB = joint.M_bodyB.M_islandIndex
	joint.M_localCenterA = joint.M_bodyA.M_sweep.LocalCenter
	joint.M_localCenterB = joint.M_bodyB.M_sweep.LocalCenter
	joint.M_invMassA = joint.M_bodyA.M_invMass
	joint.M_invMassB = joint.M_bodyB.M_invMass
	joint.M_invIA = joint.M_bodyA.M_invI
	joint.M_invIB = joint.M_bodyB.M_invI

	cA := data.Positions[joint.M_indexA].C
	aA := data.Positions[joint.M_indexA].A
	vA := data.Velocities[joint.M_indexA].V
	wA := data.Velocities[joint.M_indexA].W

	cB := data.Positions[joint.M_indexB].C
	aB := data.Positions[joint.M_indexB].A
	vB := data.Velocities[joint.M_indexB].V
	wB := data.Velocities[joint.M_indexB].W

	qA := MakeB2RotFromAngle(aA)
	qB := MakeB2RotFromAngle(aB)

	// Compute the effective masses.
	rA := B2RotVec2Mul(qA, B2Vec2Sub(joint.M_localAnchorA, joint.M_localCenterA))
	rB := B2RotVec2Mul(qB, B2Vec2Sub(joint.M_localAnchorB, joint.M_localCenterB))
	d := B2Vec2Sub(B2Vec2Add(B2Vec2Sub(cB, cA), rB), rA)

	mA := joint.M_invMassA
	mB := joint.M_invMassB
	iA := joint.M_invIA
	iB := joint.M_invIB

	// Compute motor Jacobian and effective mass.
	{
		joint.M_axis = B2RotVec2Mul(qA, joint.M_localXAxisA)
		joint.M_a1 = B2Vec2Cross(B2Vec2Add(d, rA), joint.M_axis)
		joint.M_a2 = B2Vec2Cross(rB, joint.M_axis)

		joint.M_axialMass = mA + mB + iA*joint.M_a1*joint.M_a1 + iB*joint.M_a2*joint.M_a2
		if joint.M_axialMass > 0.0 {
			joint.M_axialMass = 1.0 / joint.M_axialMass
		}
	}

	// Prismatic constraint.
	{
		joint.M_perp = B2RotVec2Mul(qA, joint.M_localYAxisA)

		joint.M_s1 = B2Vec2Cross(B2Vec2Add(d, rA), joint.M_perp)
		joint.M_s2 = B2Vec2Cross(rB, joint.M_perp)

		k11 := mA + mB + iA*joint.M_s1*joint.M_s1 + iB*joint.M_s2*joint.M_s2
		k12 := iA*joint.M_s1 + iB*joint.M_s2
		k22 := iA + iB
		if k22 == 0.0 {
			// For bodies with fixed rotation.
			k22 = 1.0
		}

		joint.M_K.Ex.Set(k11, k12)
		joint.M_K.Ey.Set(k12, k22)
	}

	if joint.M_enableLimit {
		joint.M_translation = B2Vec2Dot(joint.M_axis, d)
	} else {
		joint.M_lowerImpulse = 0.0
		joint.M_upperImpulse = 0.0
	}

	if joint.M_enableMotor == false {
		joint.M_motorImpulse = 0.0
	}

	if data.Step.WarmStarting {
		// Account for variable time step.
		joint.M_impulse.OperatorScalarMulInplace(data.Step.DtRatio)
		joint.M_motorImpulse *= data.Step.DtRatio
		joint.M_lowerImpulse *= data.Step.DtRatio
		joint.M_upperImpulse *= data.Step.DtRatio

		axialImpulse := joint.M_motorImpulse + joint.M_lowerImpulse - joint.M_upperImpulse
		P := B2Vec2Add(B2Vec2MulScalar(joint.M_impulse.X, joint.M_perp), B2Vec2MulScalar(axialImpulse, joint.M_axis))
		LA := joint.M_impulse.X*joint.M_s1 + joint.M_impulse.Y + axialImpulse*joint.M_a1
		LB := joint.M_impulse.X*joint.M_s2 + joint.M_impulse.Y + axialImpulse*joint.M_a2

		vA.OperatorMinusInplace(B2Vec2MulScalar(mA, P))
		wA -= iA * LA

		vB.OperatorPlusInplace(B2Vec2MulScalar(mB, P))
		wB += iB * LB
	} else {
		joint.M_impulse.SetZero()
		joint.M_motorImpulse = 0.0
		joint.M_lowerImpulse = 0.0
		joint.M_upperImpulse = 0.0
	}

	data.Velocities[joint.M_indexA].V = vA
	data.Velocities[joint.M_indexA].W = wA
	data.Velocities[joint.M_indexB].V = vB
	data.Velocities[joint.M_indexB].W = wB
}

func (joint *B2PrismaticJoint) SolveVelocityConstraints(data B2SolverData) {
	vA := data.Velocities[joint.M_indexA].V
	wA := data.Velocities[joint.M_indexA].W
	vB := data.Velocities[joint.M_indexB].V
	wB := data.Velocities[joint.M_indexB].W

	mA := joint.M_invMassA
	mB := joint.M_invMassB
	iA := joint.M_invIA
	iB := joint.M_invIB

	// Solve linear motor constraint
	if joint.M_enableMotor {
		Cdot := B2Vec2Dot(joint.M_axis, B2Vec2Sub(vB, vA)) + joint.M_a2*wB - joint.M_a1*wA
		impulse := joint.M_axialMass * (joint.M_motorSpeed - Cdot)
		oldImpulse := joint.M_motorImpulse
		maxImpulse := data.Step.Dt * joint.M_maxMotorForce
		joint.M_motorImpulse = B2FloatClamp(joint.M_motorImpulse+impulse, -maxImpulse, maxImpulse)
		impulse = joint.M_motorImpulse - oldImpulse

		P := B2Vec2MulScalar(impulse, joint.M_axis)
		LA := impulse * joint.M_a1
		LB := impulse * joint.M_a2

		vA.OperatorMinusInplace(B2Vec2MulScalar(mA, P))
		wA -= iA * LA
		vB.OperatorPlusInplace(B2Vec2MulScalar(mB, P))
		wB += iB * LB
	}

	if joint.M_enableLimit {
		// Lower limit
		{
			C := joint.M_translation - joint.M_lowerTranslation
			Cdot := B2Vec2Dot(joint.M_axis, B2Vec2Sub(vB, vA)) + joint.M_a2*wB - joint.M_a1*wA
			impulse := -joint.M_axialMass * (Cdot + math.Max(C, 0.0)*data.Step.Inv_dt)
			oldImpulse := joint.M_lowerImpulse
			joint.M_lowerImpulse = math.Max(joint.M_lowerImpulse+impulse, 0.0)
			impulse = joint.M_lowerImpulse - oldImpulse

			P := B2Vec2MulScalar(impulse, joint.M_axis)
			LA := impulse * joint.M_a1
			LB := impulse * joint.M_a2

			vA.OperatorMinusInplace(B2Vec2MulScalar(mA, P))
			wA -= iA * LA
			vB.OperatorPlusInplace(B2Vec2MulScalar(mB, P))
			wB += iB * LB
		}

		// Upper limit
		// Note: signs are flipped to keep C positive when the constraint is satisfied.
		// This also keeps the impulse positive when the limit is active.
		{
			C := joint.M_upperTranslation - joint.M_translation
			Cdot := B2Vec2Dot(joint.M_axis, B2Vec2Sub(vA, vB)) + joint.M_a1*wA - joint.M_a2*wB
			impulse := -joint.M_axialMass * (Cdot + math.Max(C, 0.0)*data.Step.Inv_dt)
			oldImpulse := joint.M_upperImpulse
			joint.M_upperImpulse = math.Max(joint.M_upperImpulse+impulse, 0.0)
			impulse = joint.M_upperImpulse - oldImpulse

			P := B2Vec2MulScalar(impulse, joint.M_axis)
			LA := impulse * joint.M_a1
			LB := impulse * joint.M_a2

			vA.OperatorPlusInplace(B2Vec2MulScalar(mA, P))
			wA += iA * LA
			vB.OperatorMinusInplace(B2Vec2MulScalar(mB, P))
			wB -= iB * LB
		}
	}

	// Solve the prismatic constraint in block form.
	{
		Cdot := MakeB2Vec2(0, 0)
		Cdot.X = B2Vec2Dot(joint.M_perp, B2Vec2Sub(vB, vA)) + joint.M_s2*wB - joint.M_s1*wA
		Cdot.Y = wB - wA

		df := joint.M_K.Solve(Cdot.OperatorNegate())
		joint.M_impulse.OperatorPlusInplace(df)

		P := B2Vec2MulScalar(df.X, joint.M_perp)
		LA := df.X*joint.M_s1 + df.Y
		LB := df.X*joint.M_s2 + df.Y

		vA.OperatorMinusInplace(B2Vec2MulScalar(mA, P))
		wA -= iA * LA

		vB.OperatorPlusInplace(B2Vec2MulScalar(mB, P))
		wB += iB * LB
	}

	data.Velocities[joint.M_indexA].V = vA
	data.Velocities[joint.M_indexA].W = wA
	data.Velocities[joint.M_indexB].V = vB
	data.Velocities[joint.M_indexB].W = wB
}

// A velocity based solver computes reaction forces(impulses) using the velocity constraint solver.Under this context,
// the position solver is not there to resolve forces.It is only there to cope with integration error.
//
// Therefore, the pseudo impulses in the position solver do not have any physical meaning.Thus it is okay if they suck.
//
// We could take the active state from the velocity solver.However, the joint might push past the limit when the velocity
// solver indicates the limit is inactive.
func (joint *B2PrismaticJoint) SolvePositionConstraints(data B2SolverData) bool {
	cA := data.Positions[joint.M_indexA].C
	aA := data.Positions[joint.M_indexA].A
	cB := data.Positions[joint.M_indexB].C
	aB := data.Positions[joint.M_indexB].A

	qA := MakeB2RotFromAngle(aA)
	qB := MakeB2RotFromAngle(aB)

	mA := joint.M_invMassA
	mB := joint.M_invMassB
	iA := joint.M_invIA
	iB := joint.M_invIB

	// Compute fresh Jacobians
	rA := B2RotVec2Mul(qA, B2Vec2Sub(joint.M_localAnchorA, joint.M_localCenterA))
	rB := B2RotVec2Mul(qB, B2Vec2Sub(joint.M_localAnchorB, joint.M_localCenterB))
	d := B2Vec2Sub(B2Vec2Sub(B2Vec2Add(cB, rB), cA), rA)

	axis := B2RotVec2Mul(qA, joint.M_localXAxisA)
	a1 := B2Vec2Cross(B2Vec2Add(d, rA), axis)
	a2 := B2Vec2Cross(rB, axis)
	perp := B2RotVec2Mul(qA, joint.M_localYAxisA)

	s1 := B2Vec2Cross(B2Vec2Add(d, rA), perp)
	s2 := B2Vec2Cross(rB, perp)

	impulse := MakeB2Vec3(0, 0, 0)
	C1 := MakeB2Vec2(0, 0)
	C1.X = B2Vec2Dot(perp, d)
	C1.Y = aB - aA - joint.M_referenceAngle

	linearError := math.Abs(C1.X)
	angularError := math.Abs(C1.Y)

	active := false
	C2 := 0.0
	if joint.M_enableLimit {
		translation := B2Vec2Dot(axis, d)
		if math.Abs(joint.M_upperTranslation-joint.M_lowerTranslation) < 2.0*B2_linearSlop {
			C2 = translation
			linearError = math.Max(linearError, math.Abs(translation))
			active = true
		} else if translation <= joint.M_lowerTranslation {
			C2 = math.Min(translation-joint.M_lowerTranslation, 0.0)
			linearError = math.Max(linearError, joint.M_lowerTranslation-translation)
			active = true
		} else if translation >= joint.M_upperTranslation {
			C2 = math.Max(translation-joint.M_upperTranslation, 0.0)
			linearError = math.Max(linearError, translation-joint.M_upperTranslation)
			active = true
		}
	}

	if active {
		k11 := mA + mB + iA*s1*s1 + iB*s2*s2
		k12 := iA*s1 + iB*s2
		k13 := iA*s1*a1 + iB*s2*a2
		k22 := iA + iB
		if k22 == 0.0 {
			// For fixed rotation
			k22 = 1.0
		}
		k23 := iA*a1 + iB*a2
		k33 := mA + mB + iA*a1*a1 + iB*a2*a2

		K := MakeB2Mat33()
		K.Ex.Set(k11, k12, k13)
		K.Ey.Set(k12, k22, k23)
		K.Ez.Set(k13, k23, k33)

		C := MakeB2Vec3(0, 0, 0)
		C.X = C1.X
		C.Y = C1.Y
		C.Z = C2

		impulse = K.Solve33(C.OperatorNegate())
	} else {
		k11 := mA + mB + iA*s1*s1 + iB*s2*s2
		k12 := iA*s1 + iB*s2
		k22 := iA + iB
		if k22 == 0.0 {
			k22 = 1.0
		}

		K := MakeB2Mat22()
		K.Ex.Set(k11, k12)
		K.Ey.Set(k12, k22)

		impulse1 := K.Solve(C1.OperatorNegate())
		impulse.X = impulse1.X
		impulse.Y = impulse1.Y
		impulse.Z = 0.0
	}

	P := B2Vec2Add(B2Vec2MulScalar(impulse.X, perp), B2Vec2MulScalar(impulse.Z, axis))
	LA := impulse.X*s1 + impulse.Y + impulse.Z*a1
	LB := impulse.X*s2 + impulse.Y + impulse.Z*a2

	cA.OperatorMinusInplace(B2Vec2MulScalar(mA, P))
	aA -= iA * LA
	cB.OperatorPlusInplace(B2Vec2MulScalar(mB, P))
	aB += iB * LB

	data.Positions[joint.M_indexA].C = cA
	data.Positions[joint.M_indexA].A = aA
	data.Positions[joint.M_indexB].C = cB
	data.Positions[joint.M_indexB].A = aB

	return linearError <= B2_linearSlop && angularError <= B2_angularSlop
}

func (joint B2PrismaticJoint) GetAnchorA() B2Vec2 {
	return joint.M_bodyA.GetWorldPoint(joint.M_localAnchorA)
}

func (joint B2PrismaticJoint) GetAnchorB() B2Vec2 {
	return joint.M_bodyB.GetWorldPoint(joint.M_localAnchorB)
}

func (joint B2PrismaticJoint) GetReactionForce(inv_dt float64) B2Vec2 {
	return B2Vec2MulScalar(inv_dt, B2Vec2Add(B2Vec2MulScalar(joint.M_impulse.X, joint.M_perp), B2Vec2MulScalar(joint.M_motorImpulse+joint.M_lowerImpulse+joint.M_upperImpulse, joint.M_axis)))
}

func (joint B2PrismaticJoint) GetReactionTorque(inv_dt float64) float64 {
	return inv_dt * joint.M_impulse.Y
}

func (joint B2PrismaticJoint) GetJointTranslation() float64 {
	pA := joint.M_bodyA.GetWorldPoint(joint.M_localAnchorA)
	pB := joint.M_bodyB.GetWorldPoint(joint.M_localAnchorB)
	d := B2Vec2Sub(pB, pA)
	axis := joint.M_bodyA.GetWorldVector(joint.M_localXAxisA)

	translation := B2Vec2Dot(d, axis)
	return translation
}

func (joint B2PrismaticJoint) GetJointSpeed() float64 {
	bA := joint.M_bodyA
	bB := joint.M_bodyB

	rA := B2RotVec2Mul(bA.M_xf.Q, B2Vec2Sub(joint.M_localAnchorA, bA.M_sweep.LocalCenter))
	rB := B2RotVec2Mul(bB.M_xf.Q, B2Vec2Sub(joint.M_localAnchorB, bB.M_sweep.LocalCenter))
	p1 := B2Vec2Add(bA.M_sweep.C, rA)
	p2 := B2Vec2Add(bB.M_sweep.C, rB)
	d := B2Vec2Sub(p2, p1)
	axis := B2RotVec2Mul(bA.M_xf.Q, joint.M_localXAxisA)

	vA := bA.M_linearVelocity
	vB := bB.M_linearVelocity
	wA := bA.M_angularVelocity
	wB := bB.M_angularVelocity

	speed := B2Vec2Dot(d, B2Vec2CrossScalarVector(wA, axis)) +
		B2Vec2Dot(axis, B2Vec2Sub(B2Vec2Sub(B2Vec2Add(vB, B2Vec2CrossScalarVector(wB, rB)), vA), B2Vec2CrossScalarVector(wA, rA)))
	return speed
}

func (joint B2PrismaticJoint) IsLimitEnabled() bool {
	return joint.M_enableLimit
}

func (joint *B2PrismaticJoint) EnableLimit(flag bool) {
	if flag != joint.M_enableLimit {
		joint.M_bodyA.SetAwake(true)
		joint.M_bodyB.SetAwake(true)
		joint.M_enableLimit = flag
		joint.M_lowerImpulse = 0.0
		joint.M_upperImpulse = 0.0
	}
}

func (joint B2PrismaticJoint) GetLowerLimit() float64 {
	return joint.M_lowerTranslation
}

func (joint B2PrismaticJoint) GetUpperLimit() float64 {
	return joint.M_upperTranslation
}

func (joint *B2PrismaticJoint) SetLimits(lower float64, upper float64) {
	B2Assert(lower <= upper)
	if lower != joint.M_lowerTranslation || upper != joint.M_upperTranslation {
		joint.M_bodyA.SetAwake(true)
		joint.M_bodyB.SetAwake(true)
		joint.M_lowerTranslation = lower
		joint.M_upperTranslation = upper
		joint.M_lowerImpulse = 0.0
		joint.M_upperImpulse = 0.0
	}
}

func (joint B2PrismaticJoint) IsMotorEnabled() bool {
	return joint.M_enableMotor
}

func (joint *B2PrismaticJoint) EnableMotor(flag bool) {
	if flag != joint.M_enableMotor {
		joint.M_bodyA.SetAwake(true)
		joint.M_bodyB.SetAwake(true)
		joint.M_enableMotor = flag
	}
}

func (joint *B2PrismaticJoint) SetMotorSpeed(speed float64) {
	if speed != joint.M_motorSpeed {
		joint.M_bodyA.SetAwake(true)
		joint.M_bodyB.SetAwake(true)
		joint.M_motorSpeed = speed
	}
}

func (joint *B2PrismaticJoint) SetMaxMotorForce(force float64) {
	if force != joint.M_maxMotorForce {
		joint.M_bodyA.SetAwake(true)
		joint.M_bodyB.SetAwake(true)
		joint.M_maxMotorForce = force
	}
}

func (joint B2PrismaticJoint) GetMotorForce(inv_dt float64) float64 {
	return inv_dt * joint.M_motorImpulse
}

func (joint *B2PrismaticJoint) Dump() {
	indexA := joint.M_bodyA.M_islandIndex
	indexB := joint.M_bodyB.M_islandIndex

	fmt.Printf("  b2PrismaticJointDef jd;\n")
	fmt.Printf("  jd.bodyA = bodies[%d];\n", indexA)
	fmt.Printf("  jd.bodyB = bodies[%d];\n", indexB)
	fmt.Printf("  jd.collideConnected = bool(%v);\n", joint.M_collideConnected)
	fmt.Printf("  jd.localAnchorA.Set(%.15f, %.15f);\n", joint.M_localAnchorA.X, joint.M_localAnchorA.Y)
	fmt.Printf("  jd.localAnchorB.Set(%.15f, %.15f);\n", joint.M_localAnchorB.X, joint.M_localAnchorB.Y)
	fmt.Printf("  jd.localAxisA.Set(%.15f, %.15f);\n", joint.M_localXAxisA.X, joint.M_localXAxisA.Y)
	fmt.Printf("  jd.referenceAngle = %.15f;\n", joint.M_referenceAngle)
	fmt.Printf("  jd.enableLimit = bool(%v);\n", joint.M_enableLimit)
	fmt.Printf("  jd.lowerTranslation = %.15f;\n", joint.M_lowerTranslation)
	fmt.Printf("  jd.upperTranslation = %.15f;\n", joint.M_upperTranslation)
	fmt.Printf("  jd.enableMotor = bool(%v);\n", joint.M_enableMotor)
	fmt.Printf("  jd.motorSpeed = %.15f;\n", joint.M_motorSpeed)
	fmt.Printf("  jd.maxMotorForce = %.15f;\n", joint.M_maxMotorForce)
	fmt.Printf("  joints[%d] = m_world.CreateJoint(&jd);\n", joint.M_index)
}
