package box2d

// @file
// Settings that can be overriden for your application
//

// Tunable Constants

// You can use this to change the length scale used by your game.
// For example for inches you could use 39.4.
const B2_lengthUnitsPerMeter = 1.0

// The maximum number of vertices on a convex polygon. You cannot increase
// this too much because b2BlockAllocator has a maximum object size.
const B2_maxPolygonVertices = 8
