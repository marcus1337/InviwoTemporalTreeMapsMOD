/*********************************************************************
 *  Author  : Tino Weinkauf and Wiebke Koepp
 *  Init    : Wednesday, December 06, 2017 - 23:04:14
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 *********************************************************************
 */

#pragma once

#include <modules/temporaltreemaps/temporaltreemapsmoduledefine.h>
#include <inviwo/core/common/inviwo.h>
#include <inviwo/core/processors/processor.h>
#include <modules/temporaltreemaps/datastructures/treeport.h>
#include <inviwo/core/ports/meshport.h>
#include <inviwo/core/datastructures/geometry/basicmesh.h>
#include <inviwo/core/properties/ordinalproperty.h>
#include <inviwo/core/properties/boolproperty.h>
#include <inviwo/core/properties/compositeproperty.h>

namespace inviwo
{

namespace kth
{

/** \docpage{org.inviwo.TemporalTreeMeshGenerator, Tree Mesh Generator}
    ![](org.inviwo.TemporalTreeMeshGenerator.png?classIdentifier=org.inviwo.TemporalTreeMeshGenerator)

    Explanation of how to use the processor.
    
    ### Inports
      * __<Inport1>__ <description>.
    
    ### Outports
      * __<Outport1>__ <description>.
    
    ### Properties
      * __<Prop1>__ <description>.
      * __<Prop2>__ <description>
*/


/** \class TemporalTreeMeshGenerator
    \brief VERY_BRIEFLY_DESCRIBE_THE_PROCESSOR
    
    DESCRIBE_THE_PROCESSOR_FROM_A_DEVELOPER_PERSPECTIVE

    @author Tino Weinkauf and Wiebke Koepp
*/
class IVW_MODULE_TEMPORALTREEMAPS_API TemporalTreeMeshGenerator : public Processor
{ 
//Friends
//Types
public:

//Construction / Deconstruction
public:
    TemporalTreeMeshGenerator();
    virtual ~TemporalTreeMeshGenerator() = default;

//Methods
public:
    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

    std::vector<std::shared_ptr<inviwo::IndexBufferRAM>> nonMergeVerticeIndexes;
    std::vector<std::shared_ptr<inviwo::IndexBufferRAM>> mergedVerticeIndexes;
    //void addExtras(inviwo::IndexBufferRAM& p, std::uint32_t newInd);

    typedef std::_Tuple_val <inviwo::Vector<3U, float>> vertice;
    void sliceTrianglesToTwosets(std::vector<std::shared_ptr<inviwo::IndexBufferRAM>>& indexArrays, std::shared_ptr<BasicMesh> meshBands);
    
    void turnTrianglestripToTriangles(std::vector<std::shared_ptr<inviwo::IndexBufferRAM>>& verticeIndexes);
    void sliceMergeSplitMesh(std::shared_ptr<BasicMesh> meshBands);
    void createBeziers(std::vector<BasicMesh::Vertex>& vertices, std::shared_ptr<BasicMesh> meshBands);

    struct Point {
        float x, y;
        Point(float x_, float y_) : x(x_), y(y_) {}
        Point(TemporalTreeMeshGenerator::vertice ver) : x(ver._Val.x), y(ver._Val.y) {}

        static float angleBetweenPoints(Point A, Point B, Point C) {
            float result = 0;
            float angle1 = std::atan2f(A.x - B.x, A.y - B.y);
            float angle2 = std::atan2f(C.x - B.x, C.y - B.y);
            result = angle1 - angle2;
            result = (result * 180.0) / 3.141592653589793238463;
            if (result < 0) result += 360;
            return 360.0f - std::roundf(result);
        }

        glm::vec2 getVec2() {
            return glm::vec2(x, y);
        }

        bool operator< (const Point &right) const
        {
            if (cmpf(right.x, x))
                return y < right.y;
            return x < right.x;
        }

        void mult(float multiplier) {
            x *= multiplier;
            y *= multiplier;
        }

        Point& operator+= (const Point& first) {
            x += first.x;
            y += first.y;
            return *this;
        }
        Point operator+(const Point& that) const
        {
            Point result(this->x, this->y);
            result.x += that.x;
            result.y += that.y;
            return result;
        }

        Point& operator-= (const Point& first) {
            x -= first.x;
            y -= first.y;
            return *this;
        }
        Point operator-(const Point& that) const
        {
            Point result(this->x, this->y);
            result.x -= that.x;
            result.y -= that.y;
            return result;
        }

        bool Point::operator==(const Point& A) const { return cmpf(A.x, x) && cmpf(A.y, y); }

        static bool cmpf(float A, float B, float epsilon = 0.005f)
        {
            return (fabs(A - B) < epsilon);
        }

        static float lerp(float a, float b, float f)
        {
            return (a * (1.0f - f)) + (b * f);
        }

        static std::tuple<int, int> getPointIndexes(Point elem, std::vector<std::vector<Point>>& points) {
            for (int i = 0; i < points.size(); i++) {
                int j = getPointIndex(elem, points[i]);
                if (j != -1)
                    return std::make_tuple(i, j);
            }
            return std::make_tuple(-1, -1);
        }

        static int getPointIndex(Point elem, const std::vector<Point>& points) {
            auto it = std::find(points.begin(), points.end(), elem);
            if (it == points.end())
                return -1;
            else
                return std::distance(points.begin(), it);
        }

        static Point PointLerp(Point& a, Point& b, float t) {
            Point result(0,0);
            result.x = Point::lerp(a.x, b.x, t);
            result.y = Point::lerp(a.y, b.y, t);
            return result;
        }
        static void switchTopFirst(Point& a, Point& b) {
            if (a.y < b.y) { //high Y == top.
                Point tmp = a;
                a = b;
                b = tmp;
            }
        }
        std::string getFileStr() {
            return std::to_string(x) + " " + std::to_string(y) + " ";
        }

        std::string getStr() {
            return "(" + std::to_string(x) + "," + std::to_string(y) + ") ";
        }
    };

    bool tryAddEdgeToLineSet(std::vector<std::set<Point>>& lines, Point a, Point b);
    void mergeConnectedLinesInSet(std::vector<std::set<Point>>& lines);

    void addAllCornersToLinesAndMerge(std::vector<std::vector<Point>>& corners, std::vector<std::set<Point>>& lines);

    std::vector<std::vector<Point>> makeMergeSplitCornersAndAddToLines(std::vector<BasicMesh::Vertex>& vertices, std::shared_ptr<BasicMesh> meshBands, std::vector<std::set<Point>>& linesInSets);

    std::vector<std::vector<Point>> getCornersFromMeshStartingAtIndex(std::shared_ptr<BasicMesh> meshBands, std::vector<BasicMesh::Vertex>& vertices, int index);

    std::vector < std::set < TemporalTreeMeshGenerator::Point>> createSetLines(std::vector<std::vector<TemporalTreeMeshGenerator::Point>>& allCorners);
    bool checkForDuplicatesAndErase(std::vector<std::set<Point>>& lines);
    std::vector<std::vector<Point>> makeVectorOfSetsToVectorOfVectors(std::vector<std::set<Point>>& oldLines);

    void addCornersToLine(std::vector<std::set<Point>>& lines, std::vector<Point> corners, int a, int b);
    void removeRedundantPoints(std::vector<std::vector<Point>>& lines);
    void removeIndexesFromVector(std::vector<Point>& line, std::vector<int>& pointDegrees);
    std::vector<int> calculateDegreesOfPoints(std::vector<Point>& line);
    void modifyBezierArrays(std::vector<std::vector<Point>>& allBezierArrays, std::vector<std::vector<Point>>& lines, std::vector<std::vector<Point>>& oldLines);
    
    Point findOkPointLeft(Point point, std::vector<std::vector<Point>>& oldLines, std::set<Point>& validPoints);
    Point findOkPointRight(Point point, std::vector<std::vector<Point>>& oldLines, std::set<Point>& validPoints);

    Point findOkPointLeft(int index, std::vector<Point>& oldLine, std::set<Point>& validPoints);
    Point findOkPointRight(int index, std::vector<Point>& oldLine, std::set<Point>& validPoints);

    void extractCorners(std::vector<TemporalTreeMeshGenerator::Point>& vertArr);
    void addControlPoints(std::vector<TemporalTreeMeshGenerator::Point>& vertArr);
    void storeBeziersInFile(std::vector<std::vector<Point>>& beziers, std::string fileNameTxt = "bezierinfo.txt");

    void storeOriginalTrianglesInFile(std::vector<BasicMesh::Vertex>& vertices, std::shared_ptr<BasicMesh> meshBands, std::string fileNameTxt = "originalVerts.txt");
    //void scaleTriangles(std::vector<BasicMesh::Vertex>& vertices, std::shared_ptr<BasicMesh> meshBands, float scale);

    static constexpr float curvyness = 0.5f;

protected:
    ///Our main computation function
    virtual void process() override;

    float normalTime(uint64_t time, uint64_t tMin, uint64_t tMax) const
    {
        return (time - tMin) / float(tMax - tMin);
    };

    uint64_t deNormalTime(float time, uint64_t tMin, uint64_t tMax) const
    {
        return uint64_t(time * float(tMax - tMin) + tMin);
    };

    void makeMesh(const TemporalTree& tree, std::shared_ptr<BasicMesh> meshBands, std::vector<BasicMesh::Vertex>& verticesBands,
        std::shared_ptr<BasicMesh> meshLines, std::vector<BasicMesh::Vertex>& verticesLines);

    void drawVertexPair(const float x, const float yLower, const float yUpper, const vec3& coefficients, const vec4& color,
        IndexBufferRAM& indexBufferBand, std::vector<BasicMesh::Vertex>& verticesBands);

    void drawLineVertex(const float x, const float y, IndexBufferRAM& indexBufferLine, 
        std::vector<BasicMesh::Vertex>& verticesLines);

//Ports
public:
    /// Tree for which we compute the meshes
    TemporalTreeInport portInTree;

    /// Mesh of the bands
    MeshOutport portOutMeshBands;

    /// Mesh of the bands
    MeshOutport portOutMeshLines;

//Properties
public:
    IntProperty propNumLeaves;

    CompositeProperty propTransitions;
    FloatProperty propMergeSplitBlend;

    CompositeProperty propLines;
    FloatVec4Property propColorLines;

    CompositeProperty propRenderInfo;
    BoolProperty propInterpretAsCoefficients;


//Attributes
private:

};

} // namespace kth

} // namespace
