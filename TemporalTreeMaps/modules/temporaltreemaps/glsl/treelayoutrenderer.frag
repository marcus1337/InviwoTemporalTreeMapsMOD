/*********************************************************************
 *  Author  : Tino Weinkauf and Wiebke Koepp
 *  Init    : Wednesday, November 22, 2017 - 17:26:10
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 *********************************************************************
 */

#include "utils/shading.glsl"

uniform vec4 diffuseLightColor;
uniform vec4 ambientLightColor;

uniform float x_pixels;
uniform float y_pixels;

uniform bool isLimited;
uniform vec2 p0T;
uniform vec2 p1T;
uniform vec2 p2T;
uniform vec2 p3T;

uniform vec2 p0B;
uniform vec2 p1B;
uniform vec2 p2B;
uniform vec2 p3B;

uniform vec2 d1;
uniform vec2 d2;

uniform vec2 Or0;
uniform vec2 Or1;
uniform vec2 Or2;

uniform vec2 Or00;
uniform vec2 Or11;
uniform vec2 Or22;

bool leftOfLine(vec2 p, vec2 A, vec2 B){
	if(A.x > B.x){
		vec2 tmp = A;
		A = B;
		B = tmp;
	}
	return ((B.x - A.x) * (p.y - A.y) - (B.y - A.y) * (p.x - A.x)) > 0;
}

bool isUnder(vec2 closest, vec2 nextClosest, vec2 pointPos){
        if (leftOfLine(pointPos, nextClosest, closest))
            return false;
        return true;
}

vec2 EvaluateQuadratic(vec2 a, vec2 b, vec2 c, float t){
	vec2 p0 = mix(a, b, t);
	vec2 p1 = mix(b, c, t);
	return mix(p0, p1, t);
}

vec2 EvaluateCubic(vec2 a, vec2 b, vec2 c, vec2 d, float t){
	vec2 p0 = EvaluateQuadratic(a, b, c, t);
	vec2 p1 = EvaluateQuadratic(b, c, d, t);
	return mix(p0, p1, t);
}

vec2 getClosestPoint(vec2 oldPoint, vec2 PointPos, vec2 p0, vec2 p1, vec2 p2, vec2 p3)
{
	float divisions = 50.0;
	vec2 closestPoint = p0;
	float closestDist = 999999;
	float firstDist = distance(PointPos, p0);
	if(firstDist < closestDist && p0 != oldPoint){
		closestDist = firstDist;
	}
	float t = 0;
	while(t <= 1){
		t += (1.0 /divisions);
		vec2 pointOnCurve = EvaluateCubic(p0,p1,p2,p3, t);
		float dist = distance(PointPos, pointOnCurve);
		if(dist < closestDist && pointOnCurve != oldPoint)
		{
			closestDist = dist;
			closestPoint = pointOnCurve;
		}
	}
	return closestPoint;
}

bool LiveIsUnder(vec2 pointPos, vec2 p0, vec2 p1, vec2 p2, vec2 p3){
	vec2 closestPoint = getClosestPoint(vec2(-2000.0,-2000.0), pointPos, p0, p1, p2, p3);
	vec2 nextClosestPoint = getClosestPoint(closestPoint, pointPos, p0, p1, p2, p3);
	return isUnder(closestPoint, nextClosestPoint, pointPos);
}


const float eps = 0.0000001;

vec3 getCoefficients(vec3 xValues, vec3 yValues)
{
    float diffx1x2 = xValues.x - xValues.y; // x1 - x2
    float diffx1x3 = xValues.x - xValues.z; // x1 - x3
    float diffx2x3 = xValues.y - xValues.z; // x2 - x3

    bool noDiffx1x2 = abs(diffx1x2) < eps;
    bool noDiffx1x3 = abs(diffx1x3) < eps;
    bool noDiffx2x3 = abs(diffx2x3) < eps;

    // If any of these differences are approximately 0, the result will not be a parabola
    if (noDiffx1x2 && !noDiffx1x3 && !noDiffx2x3)
        {
            // x values for x1 and x2 are approx the same, but y values are not
            if (abs(yValues.x - yValues.y) > eps)
            {
                return vec3(0.0f);
            }
            // line passing through x1 and x3
            float m = (yValues.x - yValues.z) / diffx1x3;
            float b = (-xValues.x*yValues.z + xValues.z*yValues.x) / diffx1x3;
            return vec3(0.0f, m, b);
        }
        if (!noDiffx1x2 && noDiffx1x3 && !noDiffx2x3)
        {
            // x values are approx the same for x1 and x3, but y values are not
            if (abs(yValues.x - yValues.z) > eps)
            {
                return vec3(0.0f);
            }
            // line passing through x2 and x3
            float m = (yValues.y - yValues.z) / diffx2x3;
            float b = (-xValues.y*yValues.z + xValues.z*yValues.y) / diffx2x3;
            return vec3(0.0f, m, b);
        }
        if (!noDiffx1x2 && !noDiffx1x3 && noDiffx2x3)
        {
            // x values are approx the same for x2 and x3, but y values are not
            if (abs(yValues.y - yValues.z) > eps)
            {
                return vec3(0.0f);
            }
            // line passing through x1 and x3
            float m = (yValues.x - yValues.z) / diffx1x3;
            float b = (-xValues.x*yValues.z + xValues.z*yValues.x) / diffx1x3;
            return vec3(0.0f, m, b);
        }
        if (noDiffx1x2 && noDiffx1x3 && noDiffx2x3)
        {
            if (abs(yValues.x - yValues.y) > eps
                || abs(yValues.x - yValues.z) > eps
                || abs(yValues.y - yValues.z) > eps)
            {
                return vec3(0.0f);
            }
            return vec3(0.0f, 0.0f, yValues.x);
        }
    // (x - d)(x - e) = x^2 - (d + e)x + de
    float a = yValues.x / (diffx1x2 * diffx1x3) 
        + yValues.y / (-diffx1x2 * diffx2x3) 
        + yValues.z / (-diffx1x3 * -diffx2x3);
    float b = -(xValues.y + xValues.z) * yValues.x / (diffx1x2 * diffx1x3) + 
        -(xValues.x + xValues.z) * yValues.y / (-diffx1x2 * diffx2x3) +
        -(xValues.x + xValues.y) * yValues.z / (-diffx1x3 * -diffx2x3);
    float c = (xValues.y * xValues.z) * yValues.x / (diffx1x2 * diffx1x3) +
        (xValues.x * xValues.z) * yValues.y / (-diffx1x2 * diffx2x3) +
        (xValues.x * xValues.y) * yValues.z / (-diffx1x3 * -diffx2x3);
    return vec3(a, b, c);
}

vec3 getNormalAt(vec3 coefficients, float x){
        // outer normal (0.0, 1, -(b + 2ax))
        return vec3(0.0, -coefficients.y - 2 * coefficients.x * x , 1.0);
}

in VertexData {
	vec4 color_;
	vec3 texCoord_;
	vec3 lightDir_;
	vec3 normal_;
	float vertex_;
	vec2 v;
	float leftOfDiagonal;
	
} VertexOut;

uniform bool seconddraw;

void main() {


	vec3 coeff = vec3(0.0);
#ifdef NORMAL_AS_COEFF
    coeff = VertexOut.normal_;
#endif
#ifdef NORMAL_AND_TEXTURE_AS_COEFF
    coeff = getCoefficients(VertexOut.normal_, VertexOut.texCoord_);
#endif

float tmpVert = VertexOut.vertex_;

if(isLimited == true){
	vec2 ndc_xy = gl_FragCoord.xy / vec2(x_pixels, y_pixels);
	float scaling = 1.15;
	float yDiff = abs(d1.y-d2.y)*scaling;
	float A = 0.04;
	float B = 0.005;
	
	if(!LiveIsUnder(ndc_xy, p0T, p2T, p3T, p1T)){
		discard;
	}
	
	if(LiveIsUnder(ndc_xy, p0B, p2B, p3B, p1B)){
		discard;
	}
	
	if(leftOfLine(VertexOut.v, d1, d2) && !leftOfLine(ndc_xy.xy , d1, d2)){
		discard;
	}
	
	if(!leftOfLine(VertexOut.v, d1, d2) && leftOfLine(ndc_xy.xy , d1, d2)){
		discard;
	}
	
	if(ndc_xy.x > d2.x || ndc_xy.x < d1.x){
		discard;
	}
	
	
	
	if(leftOfLine(VertexOut.v, d1, d2)){
		if(!seconddraw)
			tmpVert -= A*yDiff;
	}
	if(!leftOfLine(VertexOut.v, d1, d2)){
		if(!seconddraw)
			tmpVert += B*yDiff;
	}
	
	if(ndc_xy.y > VertexOut.v.y  && ndc_xy.x > VertexOut.v.x){
		//discard;
	}
	
}

vec3 actualNormal = getNormalAt(coeff, tmpVert);
vec3 N = normalize(actualNormal); //change normal with time?
vec3 L = normalize(VertexOut.lightDir_);

vec3 diffuseColor = max(dot(N, L),0) * VertexOut.color_.xyz * diffuseLightColor.xyz;
FragData0 = vec4(diffuseColor + ambientLightColor.xyz, 1.0);
}
