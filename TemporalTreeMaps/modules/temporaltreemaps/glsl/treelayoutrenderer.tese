
layout (triangles, equal_spacing, cw) in;

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

void main(void){ 

	VertexOut.v = (VertexIn[0].v + VertexIn[1].v + VertexIn[2].v) / 3.0;
	VertexOut.leftOfDiagonal = VertexIn[0].leftOfDiagonal;

	VertexOut.color_ = (gl_TessCoord.x * VertexIn[0].color_ + 
                     gl_TessCoord.y * VertexIn[1].color_ +
                     gl_TessCoord.z * VertexIn[2].color_);
					 
	VertexOut.texCoord_ = (gl_TessCoord.x * VertexIn[0].texCoord_ + 
                     gl_TessCoord.y * VertexIn[1].texCoord_ +
                     gl_TessCoord.z * VertexIn[2].texCoord_);
	
	
	VertexOut.lightDir_ = (gl_TessCoord.x * VertexIn[0].lightDir_ + 
                     gl_TessCoord.y * VertexIn[1].lightDir_ +
                     gl_TessCoord.z * VertexIn[2].lightDir_);
	
	
	VertexOut.normal_ = (gl_TessCoord.x * VertexIn[0].normal_ + 
                     gl_TessCoord.y * VertexIn[1].normal_ +
                     gl_TessCoord.z * VertexIn[2].normal_);
	
	
	VertexOut.vertex_ = (gl_TessCoord.x * VertexIn[0].vertex_ + 
                     gl_TessCoord.y * VertexIn[1].vertex_ +
                     gl_TessCoord.z * VertexIn[2].vertex_);

	gl_Position=(gl_TessCoord.x*gl_in[0].gl_Position+
				 gl_TessCoord.y*gl_in[1].gl_Position+
				 gl_TessCoord.z*gl_in[2].gl_Position);
	
}