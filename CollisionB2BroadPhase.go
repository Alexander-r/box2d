package box2d

type B2BroadPhaseAddPairCallback func(userDataA interface{}, userDataB interface{})

type B2Pair struct {
	ProxyIdA int
	ProxyIdB int
}

const E_nullProxy = -1

type B2BroadPhase struct {
	M_tree B2DynamicTree

	M_proxyCount int

	M_moveBuffer   []int
	M_moveCapacity int
	M_moveCount    int

	M_pairBuffer   []B2Pair
	M_pairCapacity int
	M_pairCount    int

	M_queryProxyId int
}

// Was used for sorting the pair buffer to expose duplicates:
// sort.Sort(PairByLessThan(bp.M_pairBuffer[:bp.M_pairCount]))
type PairByLessThan []B2Pair

func (a PairByLessThan) Len() int      { return len(a) }
func (a PairByLessThan) Swap(i, j int) { a[i], a[j] = a[j], a[i] }
func (a PairByLessThan) Less(i, j int) bool {
	if a[i].ProxyIdA < a[j].ProxyIdA {
		return true
	}

	if a[i].ProxyIdA == a[j].ProxyIdA {
		return a[i].ProxyIdB < a[j].ProxyIdB
	}

	return false
}

func (bp B2BroadPhase) GetUserData(proxyId int) interface{} {
	return bp.M_tree.GetUserData(proxyId)
}

func (bp B2BroadPhase) TestOverlap(proxyIdA int, proxyIdB int) bool {
	return B2TestOverlapBoundingBoxes(
		bp.M_tree.GetFatAABB(proxyIdA),
		bp.M_tree.GetFatAABB(proxyIdB),
	)
}

func (bp B2BroadPhase) GetFatAABB(proxyId int) B2AABB {
	return bp.M_tree.GetFatAABB(proxyId)
}

func (bp B2BroadPhase) GetProxyCount() int {
	return bp.M_proxyCount
}

func (bp B2BroadPhase) GetTreeHeight() int {
	return bp.M_tree.GetHeight()
}

func (bp B2BroadPhase) GetTreeBalance() int {
	return bp.M_tree.GetMaxBalance()
}

func (bp B2BroadPhase) GetTreeQuality() float64 {
	return bp.M_tree.GetAreaRatio()
}

func (bp *B2BroadPhase) UpdatePairs(addPairCallback B2BroadPhaseAddPairCallback) {
	// Reset pair buffer
	bp.M_pairCount = 0

	// Perform tree queries for all moving proxies.
	for i := 0; i < bp.M_moveCount; i++ {
		bp.M_queryProxyId = bp.M_moveBuffer[i]
		if bp.M_queryProxyId == E_nullProxy {
			continue
		}

		// We have to query the tree with the fat AABB so that
		// we don't fail to create a pair that may touch later.
		fatAABB := bp.M_tree.GetFatAABB(bp.M_queryProxyId)

		// Query tree, create pairs and add them pair buffer.
		bp.M_tree.Query(bp.QueryCallback, fatAABB)
	}

	// Send pairs to caller
	for i := 0; i < bp.M_pairCount; i++ {
		primaryPair := bp.M_pairBuffer[i]
		userDataA := bp.M_tree.GetUserData(primaryPair.ProxyIdA)
		userDataB := bp.M_tree.GetUserData(primaryPair.ProxyIdB)

		addPairCallback(userDataA, userDataB)
	}

	// Clear move flags
	for i := 0; i < bp.M_moveCount; i++ {
		proxyId := bp.M_moveBuffer[i]
		if proxyId == E_nullProxy {
			continue
		}
		bp.M_tree.ClearMoved(proxyId)
	}

	// Reset move buffer
	bp.M_moveCount = 0
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// BroadPhase.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func MakeB2BroadPhase() B2BroadPhase {

	pairCapacity := 16
	moveCapacity := 16

	tree := MakeB2DynamicTree()

	return B2BroadPhase{
		M_tree:       tree,
		M_proxyCount: 0,

		M_pairCapacity: pairCapacity,
		M_pairCount:    0,
		M_pairBuffer:   make([]B2Pair, pairCapacity),

		M_moveCapacity: moveCapacity,
		M_moveCount:    0,
		M_moveBuffer:   make([]int, moveCapacity),
	}
}

func (bp *B2BroadPhase) CreateProxy(aabb B2AABB, userData interface{}) int {
	proxyId := bp.M_tree.CreateProxy(aabb, userData)
	bp.M_proxyCount++
	bp.BufferMove(proxyId)
	return proxyId
}

func (bp *B2BroadPhase) DestroyProxy(proxyId int) {
	bp.UnBufferMove(proxyId)
	bp.M_proxyCount--
	bp.M_tree.DestroyProxy(proxyId)
}

func (bp *B2BroadPhase) MoveProxy(proxyId int, aabb B2AABB, displacement B2Vec2) {
	buffer := bp.M_tree.MoveProxy(proxyId, aabb, displacement)
	if buffer {
		bp.BufferMove(proxyId)
	}
}

func (bp *B2BroadPhase) TouchProxy(proxyId int) {
	bp.BufferMove(proxyId)
}

func (bp *B2BroadPhase) BufferMove(proxyId int) {
	if bp.M_moveCount == bp.M_moveCapacity {
		bp.M_moveBuffer = append(bp.M_moveBuffer, make([]int, bp.M_moveCapacity)...)
		bp.M_moveCapacity *= 2
	}

	bp.M_moveBuffer[bp.M_moveCount] = proxyId
	bp.M_moveCount++
}

func (bp *B2BroadPhase) UnBufferMove(proxyId int) {
	for i := 0; i < bp.M_moveCount; i++ {
		if bp.M_moveBuffer[i] == proxyId {
			bp.M_moveBuffer[i] = E_nullProxy
		}
	}
}

// This is called from b2DynamicTree::Query when we are gathering pairs.
func (bp *B2BroadPhase) QueryCallback(proxyId int) bool {

	// A proxy cannot form a pair with itself.
	if proxyId == bp.M_queryProxyId {
		return true
	}

	moved := bp.M_tree.WasMoved(proxyId)
	if moved && proxyId > bp.M_queryProxyId {
		// Both proxies are moving. Avoid duplicate pairs.
		return true
	}

	// Grow the pair buffer as needed.
	if bp.M_pairCount == bp.M_pairCapacity {
		bp.M_pairBuffer = append(bp.M_pairBuffer, make([]B2Pair, bp.M_pairCapacity)...)
		bp.M_pairCapacity = bp.M_pairCapacity + (bp.M_pairCapacity >> 1)
	}

	bp.M_pairBuffer[bp.M_pairCount].ProxyIdA = MinInt(proxyId, bp.M_queryProxyId)
	bp.M_pairBuffer[bp.M_pairCount].ProxyIdB = MaxInt(proxyId, bp.M_queryProxyId)
	bp.M_pairCount++

	return true
}

func (bp *B2BroadPhase) Query(callback B2TreeQueryCallback, aabb B2AABB) {
	bp.M_tree.Query(callback, aabb)
}

func (bp *B2BroadPhase) RayCast(callback B2TreeRayCastCallback, input B2RayCastInput) {
	bp.M_tree.RayCast(callback, input)
}

func (bp *B2BroadPhase) ShiftOrigin(newOrigin B2Vec2) {
	bp.M_tree.ShiftOrigin(newOrigin)
}
