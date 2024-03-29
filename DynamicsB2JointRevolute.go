package box2d

import (
	"fmt"
	"math"
)

// Revolute joint definition. This requires defining an anchor point where the
// bodies are joined. The definition uses local anchor points so that the
// initial configuration can violate the constraint slightly. You also need to
// specify the initial relative angle for joint limits. This helps when saving
// and loading a game.
// The local anchor points are measured from the body's origin
// rather than the center of mass because:
//  1. you might not know where the center of mass will be.
//  2. if you add/remove shapes from a body and recompute the mass,
//     the joints will be broken.
type B2RevoluteJointDef struct {
	B2JointDef

	// The local anchor point relative to bodyA's origin.
	LocalAnchorA B2Vec2

	// The local anchor point relative to bodyB's origin.
	LocalAnchorB B2Vec2

	// The bodyB angle minus bodyA angle in the reference state (radians).
	ReferenceAngle float64

	// A flag to enable joint limits.
	EnableLimit bool

	// The lower angle for the joint limit (radians).
	LowerAngle float64

	// The upper angle for the joint limit (radians).
	UpperAngle float64

	// A flag to enable the joint motor.
	EnableMotor bool

	// The desired motor speed. Usually in radians per second.
	MotorSpeed float64

	// The maximum motor torque used to achieve the desired motor speed.
	// Usually in N-m.
	MaxMotorTorque float64
}

func MakeB2RevoluteJointDef() B2RevoluteJointDef {
	res := B2RevoluteJointDef{
		B2JointDef: MakeB2JointDef(),
	}

	res.Type = B2JointType.E_revoluteJoint
	res.LocalAnchorA.Set(0.0, 0.0)
	res.LocalAnchorB.Set(0.0, 0.0)
	res.ReferenceAngle = 0.0
	res.LowerAngle = 0.0
	res.UpperAngle = 0.0
	res.MaxMotorTorque = 0.0
	res.MotorSpeed = 0.0
	res.EnableLimit = false
	res.EnableMotor = false

	return res
}

// A revolute joint constrains two bodies to share a common point while they
// are free to rotate about the point. The relative rotation about the shared
// point is the joint angle. You can limit the relative rotation with
// a joint limit that specifies a lower and upper angle. You can use a motor
// to drive the relative rotation about the shared point. A maximum motor torque
// is provided so that infinite forces are not generated.
type B2RevoluteJoint struct {
	*B2Joint

	// Solver shared
	M_localAnchorA   B2Vec2
	M_localAnchorB   B2Vec2
	M_impulse        B2Vec2
	M_motorImpulse   float64
	M_lowerImpulse   float64
	M_upperImpulse   float64
	M_enableMotor    bool
	M_maxMotorTorque float64
	M_motorSpeed     float64
	M_enableLimit    bool
	M_referenceAngle float64
	M_lowerAngle     float64
	M_upperAngle     float64

	// Solver temp
	M_indexA       int
	M_indexB       int
	M_rA           B2Vec2
	M_rB           B2Vec2
	M_localCenterA B2Vec2
	M_localCenterB B2Vec2
	M_invMassA     float64
	M_invMassB     float64
	M_invIA        float64
	M_invIB        float64
	M_K            B2Mat22
	M_angle        float64
	M_axialMass    float64
}

// The local anchor point relative to bodyA's origin.
func (joint B2RevoluteJoint) GetLocalAnchorA() B2Vec2 {
	return joint.M_localAnchorA
}

// The local anchor point relative to bodyB's origin.
func (joint B2RevoluteJoint) GetLocalAnchorB() B2Vec2 {
	return joint.M_localAnchorB
}

// Get the reference angle.
func (joint B2RevoluteJoint) GetReferenceAngle() float64 {
	return joint.M_referenceAngle
}

func (joint B2RevoluteJoint) GetMaxMotorTorque() float64 {
	return joint.M_maxMotorTorque
}

func (joint B2RevoluteJoint) GetMotorSpeed() float64 {
	return joint.M_motorSpeed
}

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Motor constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

func (def *B2RevoluteJointDef) Initialize(bA *B2Body, bB *B2Body, anchor B2Vec2) {
	def.BodyA = bA
	def.BodyB = bB
	def.LocalAnchorA = def.BodyA.GetLocalPoint(anchor)
	def.LocalAnchorB = def.BodyB.GetLocalPoint(anchor)
	def.ReferenceAngle = def.BodyB.GetAngle() - def.BodyA.GetAngle()
}

func MakeB2RevoluteJoint(def *B2RevoluteJointDef) *B2RevoluteJoint {
	res := B2RevoluteJoint{
		B2Joint: MakeB2Joint(def),
	}

	res.M_localAnchorA = def.LocalAnchorA
	res.M_localAnchorB = def.LocalAnchorB
	res.M_referenceAngle = def.ReferenceAngle

	res.M_impulse.SetZero()
	res.M_axialMass = 0.0
	res.M_motorImpulse = 0.0
	res.M_lowerImpulse = 0.0
	res.M_upperImpulse = 0.0

	res.M_lowerAngle = def.LowerAngle
	res.M_upperAngle = def.UpperAngle
	res.M_maxMotorTorque = def.MaxMotorTorque
	res.M_motorSpeed = def.MotorSpeed
	res.M_enableLimit = def.EnableLimit
	res.M_enableMotor = def.EnableMotor

	res.M_angle = 0.0

	return &res
}

func (joint *B2RevoluteJoint) InitVelocityConstraints(data B2SolverData) {
	joint.M_indexA = joint.M_bodyA.M_islandIndex
	joint.M_indexB = joint.M_bodyB.M_islandIndex
	joint.M_localCenterA = joint.M_bodyA.M_sweep.LocalCenter
	joint.M_localCenterB = joint.M_bodyB.M_sweep.LocalCenter
	joint.M_invMassA = joint.M_bodyA.M_invMass
	joint.M_invMassB = joint.M_bodyB.M_invMass
	joint.M_invIA = joint.M_bodyA.M_invI
	joint.M_invIB = joint.M_bodyB.M_invI

	aA := data.Positions[joint.M_indexA].A
	vA := data.Velocities[joint.M_indexA].V
	wA := data.Velocities[joint.M_indexA].W

	aB := data.Positions[joint.M_indexB].A
	vB := data.Velocities[joint.M_indexB].V
	wB := data.Velocities[joint.M_indexB].W

	qA := MakeB2RotFromAngle(aA)
	qB := MakeB2RotFromAngle(aB)

	joint.M_rA = B2RotVec2Mul(qA, B2Vec2Sub(joint.M_localAnchorA, joint.M_localCenterA))
	joint.M_rB = B2RotVec2Mul(qB, B2Vec2Sub(joint.M_localAnchorB, joint.M_localCenterB))

	// J = [-I -r1_skew I r2_skew]
	// r_skew = [-ry; rx]

	// Matlab
	// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x]
	//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB]

	mA := joint.M_invMassA
	mB := joint.M_invMassB
	iA := joint.M_invIA
	iB := joint.M_invIB

	joint.M_K.Ex.X = mA + mB + joint.M_rA.Y*joint.M_rA.Y*iA + joint.M_rB.Y*joint.M_rB.Y*iB
	joint.M_K.Ey.X = -joint.M_rA.Y*joint.M_rA.X*iA - joint.M_rB.Y*joint.M_rB.X*iB
	joint.M_K.Ex.Y = joint.M_K.Ey.X
	joint.M_K.Ey.Y = mA + mB + joint.M_rA.X*joint.M_rA.X*iA + joint.M_rB.X*joint.M_rB.X*iB

	joint.M_axialMass = iA + iB
	var fixedRotation bool
	if joint.M_axialMass > 0.0 {
		joint.M_axialMass = 1.0 / joint.M_axialMass
		fixedRotation = false
	} else {
		fixedRotation = true
	}

	joint.M_angle = aB - aA - joint.M_referenceAngle
	if joint.M_enableLimit == false || fixedRotation {
		joint.M_lowerImpulse = 0.0
		joint.M_upperImpulse = 0.0
	}

	if joint.M_enableMotor == false || fixedRotation {
		joint.M_motorImpulse = 0.0
	}

	if data.Step.WarmStarting {
		// Scale impulses to support a variable time step.
		joint.M_impulse.OperatorScalarMulInplace(data.Step.DtRatio)
		joint.M_motorImpulse *= data.Step.DtRatio
		joint.M_lowerImpulse *= data.Step.DtRatio
		joint.M_upperImpulse *= data.Step.DtRatio

		axialImpulse := joint.M_motorImpulse + joint.M_lowerImpulse - joint.M_upperImpulse
		P := MakeB2Vec2(joint.M_impulse.X, joint.M_impulse.Y)

		vA.OperatorMinusInplace(B2Vec2MulScalar(mA, P))
		wA -= iA * (B2Vec2Cross(joint.M_rA, P) + axialImpulse)

		vB.OperatorPlusInplace(B2Vec2MulScalar(mB, P))
		wB += iB * (B2Vec2Cross(joint.M_rB, P) + axialImpulse)
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

func (joint *B2RevoluteJoint) SolveVelocityConstraints(data B2SolverData) {
	vA := data.Velocities[joint.M_indexA].V
	wA := data.Velocities[joint.M_indexA].W
	vB := data.Velocities[joint.M_indexB].V
	wB := data.Velocities[joint.M_indexB].W

	mA := joint.M_invMassA
	mB := joint.M_invMassB
	iA := joint.M_invIA
	iB := joint.M_invIB

	fixedRotation := (iA+iB == 0.0)

	// Solve motor constraint.
	if joint.M_enableMotor && fixedRotation == false {
		Cdot := wB - wA - joint.M_motorSpeed
		impulse := -joint.M_axialMass * Cdot
		oldImpulse := joint.M_motorImpulse
		maxImpulse := data.Step.Dt * joint.M_maxMotorTorque
		joint.M_motorImpulse = B2FloatClamp(joint.M_motorImpulse+impulse, -maxImpulse, maxImpulse)
		impulse = joint.M_motorImpulse - oldImpulse

		wA -= iA * impulse
		wB += iB * impulse
	}

	if joint.M_enableLimit && fixedRotation == false {
		// Lower limit
		{
			C := joint.M_angle - joint.M_lowerAngle
			Cdot := wB - wA
			impulse := -joint.M_axialMass * (Cdot + math.Max(C, 0.0)*data.Step.Inv_dt)
			oldImpulse := joint.M_lowerImpulse
			joint.M_lowerImpulse = math.Max(joint.M_lowerImpulse+impulse, 0.0)
			impulse = joint.M_lowerImpulse - oldImpulse

			wA -= iA * impulse
			wB += iB * impulse
		}

		// Upper limit
		// Note: signs are flipped to keep C positive when the constraint is satisfied.
		// This also keeps the impulse positive when the limit is active.
		{
			C := joint.M_upperAngle - joint.M_angle
			Cdot := wA - wB
			impulse := -joint.M_axialMass * (Cdot + math.Max(C, 0.0)*data.Step.Inv_dt)
			oldImpulse := joint.M_upperImpulse
			joint.M_upperImpulse = math.Max(joint.M_upperImpulse+impulse, 0.0)
			impulse = joint.M_upperImpulse - oldImpulse

			wA += iA * impulse
			wB -= iB * impulse
		}
	}

	// Solve point-to-point constraint
	{
		Cdot := B2Vec2Sub(B2Vec2Sub(B2Vec2Add(vB, B2Vec2CrossScalarVector(wB, joint.M_rB)), vA), B2Vec2CrossScalarVector(wA, joint.M_rA))
		impulse := joint.M_K.Solve(Cdot.OperatorNegate())

		joint.M_impulse.X += impulse.X
		joint.M_impulse.Y += impulse.Y

		vA.OperatorMinusInplace(B2Vec2MulScalar(mA, impulse))
		wA -= iA * B2Vec2Cross(joint.M_rA, impulse)

		vB.OperatorPlusInplace(B2Vec2MulScalar(mB, impulse))
		wB += iB * B2Vec2Cross(joint.M_rB, impulse)
	}

	data.Velocities[joint.M_indexA].V = vA
	data.Velocities[joint.M_indexA].W = wA
	data.Velocities[joint.M_indexB].V = vB
	data.Velocities[joint.M_indexB].W = wB
}

func (joint *B2RevoluteJoint) SolvePositionConstraints(data B2SolverData) bool {
	cA := data.Positions[joint.M_indexA].C
	aA := data.Positions[joint.M_indexA].A
	cB := data.Positions[joint.M_indexB].C
	aB := data.Positions[joint.M_indexB].A

	qA := MakeB2RotFromAngle(aA)
	qB := MakeB2RotFromAngle(aB)

	angularError := 0.0
	positionError := 0.0

	fixedRotation := (joint.M_invIA+joint.M_invIB == 0.0)

	// Solve angular limit constraint
	if joint.M_enableLimit && fixedRotation == false {
		angle := aB - aA - joint.M_referenceAngle
		C := 0.0

		if math.Abs(joint.M_upperAngle-joint.M_lowerAngle) < 2.0*B2_angularSlop {
			// Prevent large angular corrections
			C = B2FloatClamp(angle-joint.M_lowerAngle, -B2_maxAngularCorrection, B2_maxAngularCorrection)
		} else if angle <= joint.M_lowerAngle {
			// Prevent large angular corrections and allow some slop.
			C = B2FloatClamp(angle-joint.M_lowerAngle+B2_angularSlop, -B2_maxAngularCorrection, 0.0)
		} else if angle >= joint.M_upperAngle {
			// Prevent large angular corrections and allow some slop.
			C = B2FloatClamp(angle-joint.M_upperAngle-B2_angularSlop, 0.0, B2_maxAngularCorrection)
		}

		limitImpulse := -joint.M_axialMass * C
		aA -= joint.M_invIA * limitImpulse
		aB += joint.M_invIB * limitImpulse
		angularError = math.Abs(C)
	}

	// Solve point-to-point constraint.
	{
		qA.Set(aA)
		qB.Set(aB)
		rA := B2RotVec2Mul(qA, B2Vec2Sub(joint.M_localAnchorA, joint.M_localCenterA))
		rB := B2RotVec2Mul(qB, B2Vec2Sub(joint.M_localAnchorB, joint.M_localCenterB))

		C := B2Vec2Sub(B2Vec2Sub(B2Vec2Add(cB, rB), cA), rA)
		positionError = C.Length()

		mA := joint.M_invMassA
		mB := joint.M_invMassB
		iA := joint.M_invIA
		iB := joint.M_invIB

		var K B2Mat22
		K.Ex.X = mA + mB + iA*rA.Y*rA.Y + iB*rB.Y*rB.Y
		K.Ex.Y = -iA*rA.X*rA.Y - iB*rB.X*rB.Y
		K.Ey.X = K.Ex.Y
		K.Ey.Y = mA + mB + iA*rA.X*rA.X + iB*rB.X*rB.X

		impulse := K.Solve(C).OperatorNegate()

		cA.OperatorMinusInplace(B2Vec2MulScalar(mA, impulse))
		aA -= iA * B2Vec2Cross(rA, impulse)

		cB.OperatorPlusInplace(B2Vec2MulScalar(mB, impulse))
		aB += iB * B2Vec2Cross(rB, impulse)
	}

	data.Positions[joint.M_indexA].C = cA
	data.Positions[joint.M_indexA].A = aA
	data.Positions[joint.M_indexB].C = cB
	data.Positions[joint.M_indexB].A = aB

	return positionError <= B2_linearSlop && angularError <= B2_angularSlop
}

func (joint B2RevoluteJoint) GetAnchorA() B2Vec2 {
	return joint.M_bodyA.GetWorldPoint(joint.M_localAnchorA)
}

func (joint B2RevoluteJoint) GetAnchorB() B2Vec2 {
	return joint.M_bodyB.GetWorldPoint(joint.M_localAnchorB)
}

func (joint B2RevoluteJoint) GetReactionForce(inv_dt float64) B2Vec2 {
	P := MakeB2Vec2(joint.M_impulse.X, joint.M_impulse.Y)
	return B2Vec2MulScalar(inv_dt, P)
}

func (joint B2RevoluteJoint) GetReactionTorque(inv_dt float64) float64 {
	return inv_dt * (joint.M_motorImpulse + joint.M_lowerImpulse - joint.M_upperImpulse)
}

func (joint B2RevoluteJoint) GetJointAngle() float64 {
	bA := joint.M_bodyA
	bB := joint.M_bodyB
	return bB.M_sweep.A - bA.M_sweep.A - joint.M_referenceAngle
}

func (joint *B2RevoluteJoint) GetJointSpeed() float64 {
	bA := joint.M_bodyA
	bB := joint.M_bodyB
	return bB.M_angularVelocity - bA.M_angularVelocity
}

func (joint B2RevoluteJoint) IsMotorEnabled() bool {
	return joint.M_enableMotor
}

func (joint *B2RevoluteJoint) EnableMotor(flag bool) {
	if flag != joint.M_enableMotor {
		joint.M_bodyA.SetAwake(true)
		joint.M_bodyB.SetAwake(true)
		joint.M_enableMotor = flag
	}
}

func (joint B2RevoluteJoint) GetMotorTorque(inv_dt float64) float64 {
	return inv_dt * joint.M_motorImpulse
}

func (joint *B2RevoluteJoint) SetMotorSpeed(speed float64) {
	if speed != joint.M_motorSpeed {
		joint.M_bodyA.SetAwake(true)
		joint.M_bodyB.SetAwake(true)
		joint.M_motorSpeed = speed
	}
}

func (joint *B2RevoluteJoint) SetMaxMotorTorque(torque float64) {
	if torque != joint.M_maxMotorTorque {
		joint.M_bodyA.SetAwake(true)
		joint.M_bodyB.SetAwake(true)
		joint.M_maxMotorTorque = torque
	}
}

func (joint B2RevoluteJoint) IsLimitEnabled() bool {
	return joint.M_enableLimit
}

func (joint *B2RevoluteJoint) EnableLimit(flag bool) {
	if flag != joint.M_enableLimit {
		joint.M_bodyA.SetAwake(true)
		joint.M_bodyB.SetAwake(true)
		joint.M_enableLimit = flag
		joint.M_lowerImpulse = 0.0
		joint.M_upperImpulse = 0.0
	}
}

func (joint B2RevoluteJoint) GetLowerLimit() float64 {
	return joint.M_lowerAngle
}

func (joint B2RevoluteJoint) GetUpperLimit() float64 {
	return joint.M_upperAngle
}

func (joint *B2RevoluteJoint) SetLimits(lower float64, upper float64) {
	B2Assert(lower <= upper)

	if lower != joint.M_lowerAngle || upper != joint.M_upperAngle {
		joint.M_bodyA.SetAwake(true)
		joint.M_bodyB.SetAwake(true)
		joint.M_lowerImpulse = 0.0
		joint.M_upperImpulse = 0.0
		joint.M_lowerAngle = lower
		joint.M_upperAngle = upper
	}
}

func (joint *B2RevoluteJoint) Dump() {
	indexA := joint.M_bodyA.M_islandIndex
	indexB := joint.M_bodyB.M_islandIndex

	fmt.Printf("  b2RevoluteJointDef jd;\n")
	fmt.Printf("  jd.bodyA = bodies[%d];\n", indexA)
	fmt.Printf("  jd.bodyB = bodies[%d];\n", indexB)
	fmt.Printf("  jd.collideConnected = bool(%v);\n", joint.M_collideConnected)
	fmt.Printf("  jd.localAnchorA.Set(%.15f, %.15f);\n", joint.M_localAnchorA.X, joint.M_localAnchorA.Y)
	fmt.Printf("  jd.localAnchorB.Set(%.15f, %.15f);\n", joint.M_localAnchorB.X, joint.M_localAnchorB.Y)
	fmt.Printf("  jd.referenceAngle = %.15f;\n", joint.M_referenceAngle)
	fmt.Printf("  jd.enableLimit = bool(%v);\n", joint.M_enableLimit)
	fmt.Printf("  jd.lowerAngle = %.15f;\n", joint.M_lowerAngle)
	fmt.Printf("  jd.upperAngle = %.15f;\n", joint.M_upperAngle)
	fmt.Printf("  jd.enableMotor = bool(%v);\n", joint.M_enableMotor)
	fmt.Printf("  jd.motorSpeed = %.15f;\n", joint.M_motorSpeed)
	fmt.Printf("  jd.maxMotorTorque = %.15f;\n", joint.M_maxMotorTorque)
	fmt.Printf("  joints[%d] = m_world.CreateJoint(&jd);\n", joint.M_index)
}

//void b2RevoluteJoint::Draw(b2Draw* draw) const
//{
//	const b2Transform& xfA = m_bodyA->GetTransform();
//	const b2Transform& xfB = m_bodyB->GetTransform();
//	b2Vec2 pA = b2Mul(xfA, m_localAnchorA);
//	b2Vec2 pB = b2Mul(xfB, m_localAnchorB);
//
//	b2Color c1(0.7f, 0.7f, 0.7f);
//	b2Color c2(0.3f, 0.9f, 0.3f);
//	b2Color c3(0.9f, 0.3f, 0.3f);
//	b2Color c4(0.3f, 0.3f, 0.9f);
//	b2Color c5(0.4f, 0.4f, 0.4f);
//
//	draw->DrawPoint(pA, 5.0f, c4);
//	draw->DrawPoint(pB, 5.0f, c5);
//
//	float aA = m_bodyA->GetAngle();
//	float aB = m_bodyB->GetAngle();
//	float angle = aB - aA - m_referenceAngle;
//
//	const float L = 0.5f;
//
//	b2Vec2 r = L * b2Vec2(cosf(angle), sinf(angle));
//	draw->DrawSegment(pB, pB + r, c1);
//	draw->DrawCircle(pB, L, c1);
//
//	if (m_enableLimit)
//	{
//		b2Vec2 rlo = L * b2Vec2(cosf(m_lowerAngle), sinf(m_lowerAngle));
//		b2Vec2 rhi = L * b2Vec2(cosf(m_upperAngle), sinf(m_upperAngle));
//
//		draw->DrawSegment(pB, pB + rlo, c2);
//		draw->DrawSegment(pB, pB + rhi, c3);
//	}
//
//	b2Color color(0.5f, 0.8f, 0.8f);
//	draw->DrawSegment(xfA.p, pA, color);
//	draw->DrawSegment(pA, pB, color);
//	draw->DrawSegment(xfB.p, pB, color);
//}
