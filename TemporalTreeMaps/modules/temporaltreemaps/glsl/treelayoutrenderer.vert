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

out VertexData {
	vec4 color_;
	vec3 texCoord_;
	vec3 lightDir_;
	vec3 normal_;
	float vertex_;
} VertexIn;
 
void main() {
    VertexIn.color_ = in_Color;
    VertexIn.texCoord_ = in_TexCoord;
    VertexIn.lightDir_ = lightPos;
    VertexIn.normal_ = in_Normal;
    VertexIn.vertex_ = in_Vertex.y;
    gl_Position = projectionMatrix * in_Vertex; 
}