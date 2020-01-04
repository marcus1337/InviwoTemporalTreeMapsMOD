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

uniform bool seconddraw;


in VertexData {
	vec4 color_;
	vec3 texCoord_;
	vec3 lightDir_;
	vec3 normal_;
	float vertex_;
	vec2 v;
	float leftOfDiagonal;
} VertexIn[];
 
out VertexData {
	vec4 color_;
	vec3 texCoord_;
	vec3 lightDir_;
	vec3 normal_;
	float vertex_;
	vec2 v;
	float leftOfDiagonal;
	
} VertexOut;

void main(void)
{
	vec2 pos[3];
	
	for(int i = 0; i < 3; i++){
		pos[i] = vec2(gl_in[i].gl_Position);
	}
	float centerX1 = (pos[0].x + pos[1].x + pos[2].x) / 3.0;
	float centerY1 = (pos[0].y + pos[1].y + pos[2].y) / 3.0;
	vec2 cent = vec2(centerX1,centerY1);
	
	float multVal = 1.2;
	
	if(seconddraw){
		multVal = 1.0;
	}
	
	for(int i = 0; i < 3; i++){
		pos[i] -= cent;
		pos[i] *= multVal;
		pos[i] += cent;
	}
	VertexOut.v = (VertexIn[0].v + VertexIn[1].v + VertexIn[2].v) / 3.0;
	VertexOut.leftOfDiagonal = VertexIn[0].leftOfDiagonal;

	for(int i = 0; i < gl_in.length(); i++)
	{
		if(!seconddraw){
			VertexOut.color_ = VertexIn[i].color_;
			VertexOut.texCoord_ = VertexIn[i].texCoord_;
			VertexOut.lightDir_ = VertexIn[i].lightDir_ ;
			VertexOut.normal_ = VertexIn[i].normal_*0.83;
			VertexOut.vertex_ = VertexIn[i].vertex_*0.83;
			gl_Position = vec4(pos[i], gl_in[i].gl_Position.z, gl_in[i].gl_Position.w);
		}else{
			VertexOut.color_ = VertexIn[i].color_;
			VertexOut.texCoord_ = VertexIn[i].texCoord_;
			VertexOut.lightDir_ = VertexIn[i].lightDir_ ;
			VertexOut.normal_ = VertexIn[i].normal_ ;
			VertexOut.vertex_ = VertexIn[i].vertex_ ;
			gl_Position = vec4(pos[i], gl_in[i].gl_Position.z, gl_in[i].gl_Position.w);
		}

		EmitVertex();
	}
}
