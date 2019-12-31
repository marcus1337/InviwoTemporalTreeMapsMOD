/*********************************************************************
 *  Author  : Tino Weinkauf and Wiebke Koepp
 *  Init    : Wednesday, November 22, 2017 - 17:26:10
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 *********************************************************************
 */
 
#ifndef GLSL_VERSION_150
#extension GL_EXT_gpu_shader4 : enable
#extension GL_EXT_geometry_shader4 : enable
#endif

layout(triangles) in;
layout(triangle_strip, max_vertices = 3) out;
//layout(line_strip, max_vertices = 3) out;


in VertexData {
	vec4 color_;
	vec3 texCoord_;
	vec3 lightDir_;
	vec3 normal_;
	float vertex_;
} VertexIn[];
 
out VertexData {
	vec4 color_;
	vec3 texCoord_;
	vec3 lightDir_;
	vec3 normal_;
	float vertex_;
} VertexOut;

void main(void)
{
	for(int i = 0; i < gl_in.length(); i++)
	{
		VertexOut.color_ = VertexIn[i].color_;
		VertexOut.texCoord_ = VertexIn[i].texCoord_;
		VertexOut.lightDir_ = VertexIn[i].lightDir_;
		VertexOut.normal_ = VertexIn[i].normal_;
		VertexOut.vertex_ = VertexIn[i].vertex_;
		gl_Position = gl_in[i].gl_Position;	
		EmitVertex();
	}
}
