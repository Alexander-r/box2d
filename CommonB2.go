package box2d

import "math"

// @port(OK)
const B2DEBUG = false

// @port(OK)
func B2Assert(a bool) {
	if !a {
		panic("B2Assert")
	}
}

const B2_maxFloat = math.MaxFloat64
const B2_epsilon = math.SmallestNonzeroFloat64
const B2_pi = math.Pi

// @file
// Global tuning constants based on meters-kilograms-seconds (MKS) units.
//

// Collision

// The maximum number of contact points between two convex shapes. Do
// not change this value.
const B2_maxManifoldPoints = 2

// This is used to fatten AABBs in the dynamic tree. This allows proxies
// to move by a small amount without triggering a tree adjustment.
// This is in meters.
const B2_aabbExtension = 0.1 * B2_lengthUnitsPerMeter

// This is used to fatten AABBs in the dynamic tree. This is used to predict
// the future position based on the current displacement.
// This is a dimensionless multiplier.
const B2_aabbMultiplier = 4.0 * B2_lengthUnitsPerMeter

// A small length used as a collision and constraint tolerance. Usually it is
// chosen to be numerically significant, but visually insignificant.
const B2_linearSlop = 0.005 * B2_lengthUnitsPerMeter

// A small angle used as a collision and constraint tolerance. Usually it is
// chosen to be numerically significant, but visually insignificant.
const B2_angularSlop = (2.0 / 180.0 * B2_pi)

// The radius of the polygon/edge shape skin. This should not be modified. Making
// this smaller means polygons will have an insufficient buffer for continuous collision.
// Making it larger may create artifacts for vertex collision.
const B2_polygonRadius = (2.0 * B2_linearSlop)

// Maximum number of sub-steps per contact in continuous physics simulation.
const B2_maxSubSteps = 8

// Dynamics

// Maximum number of contacts to be handled to solve a TOI impact.
const B2_maxTOIContacts = 32

// The maximum linear position correction used when solving constraints. This helps to
// prevent overshoot.
const B2_maxLinearCorrection = 0.2 * B2_lengthUnitsPerMeter

// The maximum angular position correction used when solving constraints. This helps to
// prevent overshoot.
const B2_maxAngularCorrection = (8.0 / 180.0 * B2_pi)

// The maximum linear translation of a body per step. This limit is very large and is used
// to prevent numerical problems. You shouldn't need to adjust this. Meters.
const B2_maxTranslation = 2.0 * B2_lengthUnitsPerMeter
const B2_maxTranslationSquared = (B2_maxTranslation * B2_maxTranslation)

// The maximum angular velocity of a body. This limit is very large and is used
// to prevent numerical problems. You shouldn't need to adjust this.
const B2_maxRotation = (0.5 * B2_pi)
const B2_maxRotationSquared = (B2_maxRotation * B2_maxRotation)

// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
// that overlap is removed in one time step. However using values close to 1 often lead
// to overshoot.
const B2_baumgarte = 0.2
const B2_toiBaumgarte = 0.75

// Sleep

// The time that a body must be still before it will go to sleep.
const B2_timeToSleep = 0.5

// A body cannot sleep if its linear velocity is above this tolerance.
const B2_linearSleepTolerance = 0.01 * B2_lengthUnitsPerMeter

// A body cannot sleep if its angular velocity is above this tolerance.
const B2_angularSleepTolerance = (2.0 / 180.0 * B2_pi)

// Version numbering scheme.
type B2Version struct {
	// significant changes
	Major int32
	// incremental changes
	Minor int32
	// bug fixes
	Revision int32
}

var B2_version B2Version = B2Version{2, 4, 1}
