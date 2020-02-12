package box2d_test

import (
	"fmt"
	"github.com/ByteArena/box2d"
	"github.com/pmezard/go-difflib/difflib"
	"testing"
)

var expectedShapeCast string = "hit = false, iters = 3, lambda = 1, distance = 7.040063920164362"

func TestCPPComplianceShapeCast(t *testing.T) {
	transformA := box2d.MakeB2Transform()
	transformA.P = box2d.MakeB2Vec2(0.0, 0.25)
	transformA.Q.SetIdentity()

	transformB := box2d.MakeB2Transform()
	transformB.SetIdentity()

	input := box2d.MakeB2ShapeCastInput()

	pA := box2d.MakeB2DistanceProxy()
	pA.M_vertices = append(pA.M_vertices, box2d.MakeB2Vec2(-0.5, 1.0))
	pA.M_vertices = append(pA.M_vertices, box2d.MakeB2Vec2(0.5, 1.0))
	pA.M_vertices = append(pA.M_vertices, box2d.MakeB2Vec2(0.0, 0.0))
	pA.M_count = 3
	pA.M_radius = box2d.B2_polygonRadius

	pB := box2d.MakeB2DistanceProxy()
	pB.M_vertices = append(pB.M_vertices, box2d.MakeB2Vec2(-0.5, -0.5))
	pB.M_vertices = append(pB.M_vertices, box2d.MakeB2Vec2(0.5, -0.5))
	pB.M_vertices = append(pB.M_vertices, box2d.MakeB2Vec2(0.5, 0.5))
	pB.M_vertices = append(pB.M_vertices, box2d.MakeB2Vec2(-0.5, 0.5))
	pB.M_count = 4
	pB.M_radius = box2d.B2_polygonRadius

	input.ProxyA = pA
	input.ProxyB = pB
	input.TransformA = transformA
	input.TransformB = transformB
	input.TranslationB.Set(8.0, 0.0)

	output := box2d.MakeB2ShapeCastOutput()

	hit := box2d.B2ShapeCast(&output, &input)

	transformB2 := box2d.MakeB2Transform()
	transformB2.Q = transformB.Q
	transformB2.P = box2d.B2Vec2Add(transformB.P, box2d.B2Vec2MulScalar(output.Lambda, input.TranslationB))

	distanceInput := box2d.MakeB2DistanceInput()
	distanceInput.ProxyA = pA
	distanceInput.ProxyB = pB
	distanceInput.TransformA = transformA
	distanceInput.TransformB = transformB2
	distanceInput.UseRadii = false
	simplexCache := box2d.MakeB2SimplexCache()
	simplexCache.Count = 0
	distanceOutput := box2d.MakeB2DistanceOutput()

	box2d.B2Distance(&distanceOutput, &simplexCache, &distanceInput)

	msg := fmt.Sprintf("hit = %v, iters = %v, lambda = %v, distance = %.15f",
		hit, output.Iterations, output.Lambda, distanceOutput.Distance)

	fmt.Println(msg)

	if msg != expectedShapeCast {
		diff := difflib.UnifiedDiff{
			A:        difflib.SplitLines(expectedShapeCast),
			B:        difflib.SplitLines(msg),
			FromFile: "Expected",
			ToFile:   "Current",
			Context:  0,
		}
		text, _ := difflib.GetUnifiedDiffString(diff)
		t.Fatalf("NOT Matching c++ reference. Failure: \n%s", text)
	}
}

var expectedShapeCast2 string = "hit = true, iters = 20, lambda = 0, distance = 0.250000000000000"

func TestCPPComplianceShapeCast2(t *testing.T) {
	transformA := box2d.MakeB2Transform()
	transformA.P = box2d.MakeB2Vec2(0.0, 0.25)
	transformA.Q.SetIdentity()

	transformB := box2d.MakeB2Transform()
	transformB.SetIdentity()

	input := box2d.MakeB2ShapeCastInput()

	pA := box2d.MakeB2DistanceProxy()
	pA.M_vertices = append(pA.M_vertices, box2d.MakeB2Vec2(0.0, 0.0))
	pA.M_count = 1
	pA.M_radius = 0.5

	pB := box2d.MakeB2DistanceProxy()
	pB.M_vertices = append(pB.M_vertices, box2d.MakeB2Vec2(0.0, 0.0))
	pB.M_count = 1
	pB.M_radius = 0.5

	input.ProxyA = pA
	input.ProxyB = pB
	input.TransformA = transformA
	input.TransformB = transformB
	input.TranslationB.Set(8.0, 0.0)

	output := box2d.MakeB2ShapeCastOutput()

	hit := box2d.B2ShapeCast(&output, &input)

	transformB2 := box2d.MakeB2Transform()
	transformB2.Q = transformB.Q
	transformB2.P = box2d.B2Vec2Add(transformB.P, box2d.B2Vec2MulScalar(output.Lambda, input.TranslationB))

	distanceInput := box2d.MakeB2DistanceInput()
	distanceInput.ProxyA = pA
	distanceInput.ProxyB = pB
	distanceInput.TransformA = transformA
	distanceInput.TransformB = transformB2
	distanceInput.UseRadii = false
	simplexCache := box2d.MakeB2SimplexCache()
	simplexCache.Count = 0
	distanceOutput := box2d.MakeB2DistanceOutput()

	box2d.B2Distance(&distanceOutput, &simplexCache, &distanceInput)

	msg := fmt.Sprintf("hit = %v, iters = %v, lambda = %v, distance = %.15f",
		hit, output.Iterations, output.Lambda, distanceOutput.Distance)

	fmt.Println(msg)

	if msg != expectedShapeCast2 {
		diff := difflib.UnifiedDiff{
			A:        difflib.SplitLines(expectedShapeCast2),
			B:        difflib.SplitLines(msg),
			FromFile: "Expected",
			ToFile:   "Current",
			Context:  0,
		}
		text, _ := difflib.GetUnifiedDiffString(diff)
		t.Fatalf("NOT Matching c++ reference. Failure: \n%s", text)
	}
}
