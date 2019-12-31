/*********************************************************************
 *  Author  : Tino Weinkauf and Wiebke Koepp
 *  Init    : Wednesday, November 22, 2017 - 17:26:10
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 *********************************************************************
 */

#include <modules/temporaltreemaps/processors/treelayoutrenderer.h>
#include <modules/temporaltreemaps/datastructures/treeorder.h>
#include <modules/opengl/texture/textureutils.h>
#include <modules/opengl/shader/shaderutils.h>
#include <modules/opengl/openglutils.h>
#include <inviwo/core/rendering/meshdrawerfactory.h>
#include <inviwo/core/rendering/meshdrawer.h>

namespace inviwo
{
namespace kth
{

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo TemporalTreeLayoutRenderer::processorInfo_
{
    "org.inviwo.TemporalTreeLayoutRenderer",      // Class identifier
    "Tree Layout Renderer",                // Display name
    "Temporal Tree",              // Category
    CodeState::Experimental,  // Code state
    Tags::None,               // Tags
};

const ProcessorInfo TemporalTreeLayoutRenderer::getProcessorInfo() const
{
    return processorInfo_;
}

typedef std::pair<ShaderType, std::string> par;

TemporalTreeLayoutRenderer::TemporalTreeLayoutRenderer()
    : Processor()
    , portInMeshBands("inMeshBands")
    , portInMeshLines("inOrder")
    , portInImage("inImage")
    , portOutImage("outImage")
    , top("top", "Top Margin", 0, 0, 1)
    , bottom("bottom", "Bottom Margin", 0, 0, 1)
    , left("left", "Left Margin", 0, 0, 1)
    , right("right", "Right Margin", 0, 0, 1)
    , propLightPosition("lightPos", "Light Direction", vec3(0, 0, 1), vec3(-2, -2, -10), vec3(2, 2, 10))
    , propInterpretAsCoefficients("coeffInterpret", "Normal is Coefficient", false)
    , propAmbientLight("ambientLight", "Ambient", vec4(0.0f, 0.0f, 0.0f, 1.0f),
        vec4(0.0f), vec4(1.0f), vec4(0.1f), InvalidationLevel::InvalidOutput, PropertySemantics::Color)
    , propDiffuseLight("diffuseLight", "Diffuse", vec4(0.0f, 0.0f, 0.0f, 1.0f),
        vec4(0.0f), vec4(1.0f), vec4(0.1f), InvalidationLevel::InvalidOutput, PropertySemantics::Color)
    , propAxes("axes", "Axes")
    , propXAxis("xAxis", "X Axis")
	, propXMargin("xMargin", "X Margin", 0.5f, 0.0f, 1.0f)
    , propYAxis("yAxis", "Y Axis", plot::AxisProperty::Orientation::Vertical)
    , propYMargin("yMargin", "Y Margin", 0.5f, 0.0f, 1.0f)
	//, bandShader("treelayoutrenderer.vert", "treelayoutrenderer.geom", "treelayoutrenderer.frag")
    , bandShader({
        par(ShaderType::Vertex, "treelayoutrenderer.vert"),
        par(ShaderType::TessellationControl, "treelayoutrenderer.tesc"),
        par(ShaderType::TessellationEvaluation, "treelayoutrenderer.tese"),
        par(ShaderType::Geometry, "treelayoutrenderer.geom"),
        par(ShaderType::Fragment, "treelayoutrenderer.frag") })
	, lineShader("treelinerenderer.vert", "treelinerenderer.frag")
    , axisRenderers({ { propXAxis, propYAxis } })
{
  //  bandShader = Shader("treelayoutrenderer.vert", "treelayoutrenderer.geom", "treelayoutrenderer.frag");
    // Ports
    addPort(portInMeshBands);
    addPort(portInMeshLines);
    addPort(portInImage);
    addPort(portOutImage);

    // Yes?
    portInImage.setOptional(true);
    portInMeshLines.setOptional(true);

    // Properties
    addProperty(left);
    addProperty(right);
    addProperty(bottom);
    addProperty(top);

    addProperty(propAxes);
    propAxes.addProperty(propXAxis);
    propAxes.addProperty(propXMargin);
    propAxes.addProperty(propYAxis);
    propAxes.addProperty(propYMargin);
    
    addProperty(propLightPosition);
    addProperty(propAmbientLight);
    addProperty(propDiffuseLight);

    addProperty(propInterpretAsCoefficients);

    portInMeshBands.onChange([this]() { updateMeshs(); });
    portInMeshLines.onChange([this]() { updateMeshs(); });
}

void TemporalTreeLayoutRenderer::updateMeshs()
{
    
    using InputData = std::pair<inviwo::Outport*, std::shared_ptr<const inviwo::Mesh>>;

    // Get the vector of all inputs.
    std::vector<InputData> bandData = portInMeshBands.getSourceVectorData();
    std::vector<InputData> lineData = portInMeshLines.getSourceVectorData();

    // 
    bool (*notNull)(const InputData&) = [](const InputData& other) { return other.second == nullptr; };

    // Find the first one that is not nullptr.
    InputData bandMeshData =
        *std::find_if_not(
            bandData.begin(),
            bandData.end(),
            notNull);

    auto lineMeshDataPtr =
        std::find_if_not(
            lineData.begin(),
            lineData.end(),
            notNull);
    

    // Seems like we need this fatory to load meshes.
    auto factory = getNetwork()->getApplication()->getMeshDrawerFactory();

    // Set new band mesh if data is valid.
    if (auto renderer = factory->create(bandMeshData.second.get()))
        bandMesh = std::move(renderer);

    lineMesh = nullptr;
    // Set new line mesh if data is valid.
    if (lineMeshDataPtr != lineData.end())
        if (auto renderer = factory->create(lineMeshDataPtr->second.get()))
            lineMesh = std::move(renderer);
}

void TemporalTreeLayoutRenderer::process()
{
    if (portInImage.isConnected()) {
        utilgl::activateTargetAndCopySource(portOutImage, portInImage, ImageType::ColorDepth);
    }
    else {
        utilgl::activateAndClearTarget(portOutImage, ImageType::ColorDepth);
    }
    auto imageDimensions = portInImage.getData()->getDimensions();
  //  portInImage.getData()->get
    /*std::fstream fs;
    fs.open("testABC000.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    fs << "TEST: "
        << "X: " << test.r << " | Y: " << test.g << " | " << " | " << test.x << "\n";
    fs.close();*/

    mat4 proj = glm::ortho(0.0f - left.get(), 1.0f + right.get(), 0.0f - bottom.get(), 1.0f + top.get(), -200.0f, 100.0f);

    {
        if (propInterpretAsCoefficients.get())
        {
         
            bandShader.getFragmentShaderObject()->addShaderDefine("NORMAL_AS_COEFF");
            bandShader.getFragmentShaderObject()->removeShaderDefine("NORMAL_AND_TEXTURE_AS_COEFF");
        }else
        {
            bandShader.getFragmentShaderObject()->removeShaderDefine("NORMAL_AS_COEFF");
            bandShader.getFragmentShaderObject()->addShaderDefine("NORMAL_AND_TEXTURE_AS_COEFF");
        }
       
       
        bandShader.build();        

        bandShader.activate();

        bandShader.setUniform("projectionMatrix", proj);
        bandShader.setUniform("lightPos", propLightPosition);
        bandShader.setUniform("ambientLightColor", propAmbientLight);
        bandShader.setUniform("diffuseLightColor", propDiffuseLight);
        glm::float1 xpixels = imageDimensions.r;
        glm::float1 ypixels = imageDimensions.g;

        glm::float1 x_width = 941;
        glm::float1 y_height = 646;

        bandShader.setUniform("x_width", x_width);
        bandShader.setUniform("y_height", y_height);
       
        bandShader.setUniform("x_pixels", xpixels);
        bandShader.setUniform("y_pixels", ypixels);

       /* glm::bool1 isTest = true;
        glm::vec2 testVert(0.1, 0.1);
        bandShader.setUniform("testVert", testVert);
        bandShader.setUniform("isTest", isTest);*/


        utilgl::GlBoolState depthTest(GL_DEPTH_TEST, false);
        glEnable(GL_PROGRAM_POINT_SIZE);

        // Set all band shader uniforms and draw.
        if (bandMesh.get())
        {
            //utilgl::setShaderUniforms(bandShader, *(bandMesh->getMesh()), "geometry_");
            bandMesh->bandShader = &bandShader;
            bandMesh->drawInfo = 999;
            bandMesh->draw();
        }

        bandShader.deactivate();
    }
    // Don't know why. Seems to need this.
    utilgl::deactivateCurrentTarget();
    utilgl::activateTarget(portOutImage, ImageType::ColorDepth);
    {
        lineShader.activate();
        lineShader.setUniform("projectionMatrix", proj);

        // Set all line shader uniforms and draw.
        if (lineMesh.get())
        {
            utilgl::setShaderUniforms(lineShader, *(lineMesh->getMesh()), "geometry_");
            lineMesh->draw();
        }

        lineShader.deactivate();
    }

    const size2_t dims = portOutImage.getDimensions();

    float yRange = 1.0f + top + bottom;
    float xRange = 1.0f + left + right;

    // draw horizontal axis (x)
    axisRenderers[0].render(dims, size2_t(left/xRange * dims.x, propXMargin* bottom/yRange * dims.y),
        size2_t(dims.x - right/xRange *dims.x, propXMargin* bottom / yRange * dims.y));
    
    
    // draw vertical axis (y)
    axisRenderers[1].render(dims, size2_t(propYMargin * left / xRange * dims.x, bottom / yRange * dims.y),
        size2_t(propYMargin * left / xRange * dims.x, dims.y - top / yRange * dims.y));


    utilgl::deactivateCurrentTarget();
}

} // namespace kth
} // namespace

