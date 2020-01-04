/*********************************************************************
 *  Author  : Tino Weinkauf and Wiebke Koepp
 *  Init    : Wednesday, November 22, 2017 - 17:26:10
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 *********************************************************************
 */

#include "utils/structs.glsl"

uniform GeometryParameters geometry_;
uniform mat4 projectionMatrix;
uniform vec3 lightPos;

uniform vec2 d1;
uniform vec2 d2;

out VertexData {
	vec4 color_;
	vec3 texCoord_;
	vec3 lightDir_;
	vec3 normal_;
	float vertex_;
	vec2 v;
	
	float leftOfDiagonal;
} VertexIn;


 
void main() {
	VertexIn.leftOfDiagonal = 0.0;
	
	VertexIn.v = in_Vertex.xy;
	
    VertexIn.color_ = in_Color;
    VertexIn.texCoord_ = in_TexCoord;
    VertexIn.lightDir_ = lightPos;
    VertexIn.normal_ = in_Normal;
    VertexIn.vertex_ = in_Vertex.y;
	
    gl_Position = projectionMatrix * in_Vertex; 
}