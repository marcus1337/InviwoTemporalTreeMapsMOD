layout (vertices = 3) out;

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
} VertexOut[];

void main(void){
    if (gl_InvocationID == 0){
		gl_TessLevelInner[0] = 1.0;
        gl_TessLevelOuter[0] = 1.0;
        gl_TessLevelOuter[1] = 1.0;
        gl_TessLevelOuter[2] = 1.0;
	
        /*gl_TessLevelInner[0] = 7.0;
        gl_TessLevelOuter[0] = 2.0;
        gl_TessLevelOuter[1] = 3.0;
        gl_TessLevelOuter[2] = 7.0;*/
    } 
    gl_out[gl_InvocationID].gl_Position = gl_in[gl_InvocationID].gl_Position;
	VertexOut[gl_InvocationID].color_ = VertexIn[gl_InvocationID].color_;
	VertexOut[gl_InvocationID].texCoord_ = VertexIn[gl_InvocationID].texCoord_;
	VertexOut[gl_InvocationID].lightDir_ = VertexIn[gl_InvocationID].lightDir_;
	VertexOut[gl_InvocationID].normal_ = VertexIn[gl_InvocationID].normal_;
	VertexOut[gl_InvocationID].vertex_ = VertexIn[gl_InvocationID].vertex_;
	VertexOut[gl_InvocationID].v = VertexIn[gl_InvocationID].v;
	VertexOut[gl_InvocationID].leftOfDiagonal = VertexIn[gl_InvocationID].leftOfDiagonal;
	
}