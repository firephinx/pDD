#pragma once
#include <vector>
#include <boost/unordered_set.hpp>

#include <units.h>
#include <utils.h>
#include <map>
#include "opengl_render.h"

using namespace std;

typedef boost::uniform_real<px> uniform_real_dist;

typedef boost::variate_generator<RNG_ &, boost::uniform_real<px> >
        uniform_real_variate;



class Mesh{
public:
    // Data members
    vector< SpaceCoord > vertices_;
    vector< int > triangles_;
    vector< ImCoord > uvs_;
    vector< int > uv_triangles_;
    //vector< vector< int > > vertex_face_neighbors_;
    vector< int > texture_assignment_;
    vector<int> uv_rows_;
    vector<int> uv_cols_;
    vector<image_color> vertex_colors_;
    vector<cv::Mat> textures_;
    vector<Material> materials_;
    bool mtlFound;
    OBJModel objModel;
public:
    
    // Constructor
    //Mesh();
    //Mesh(string filepath);
    
    // Query Methods
    size_t TriangleCount() const;
    SpaceCoord TriangleVertex(int triangle, int vertex) const;
    ImCoord NormalizedTriangleTextureVertex(int triangle, int vertex) const;   
    ImCoord TriangleTextureVertex(int triangle, int vertex) const;
    int TextureAssignment(int triangle_index) const;
    
    // Query methods that require computation
    ImCoord NormalizeUv(ImCoord pt, int texture_index) const;
    ImCoord GetDenormalizedUv(int uv_index, int triangle_index) const;
    void TriangleAreas(vector< cm >& areas) const;
    void TriangleUvAreas(vector< px >& areas) const;
    void TriangleAreasPerTexture(int num_textures, vector< cm >& areas) const;
    cm TotalArea() const;
    //void FindEnclosingUvTriangle(
    //        ImCoord pt, ind& triangle, Barycentric& bary) const;
    Barycentric UvToBarycentric(ImCoord pt, int triangle) const;
    SpaceCoord BarycentricToWorld(Barycentric, int) const;
    ImCoord BarycentricToUv(Barycentric, int) const;
    SpaceCoord TriangleNormal(int triangle) const;
    void GetTrianglesWithinUvRadius(
            ImCoord center,
            int center_triangle,
            px radius,
            boost::unordered_set<int>&) const;
    void PxPerCm(vector<px>& px_per_cm) const;
    cv::Mat GenerateCoverageMask(cv::Mat, int) const;
    cv::Mat Generate3DMask(cv::Mat image, Pose pose, Intrinsics intrinsics);
    //cv::Mat GeneratePxPerCmLookup(cv::Mat, ind) const;
    cv::Mat GenerateTriangleLookup(cv::Mat, int) const;
    cv::Mat GeneratePosedTriangleLookup(int rows,
                                        int cols,
                                        Pose pose,
                                        Intrinsics intrinsics) const;
    bool TriangleRayIntersect(Pose pose,
                              SpaceCoord point,
                              SpaceCoord direction,
                              int triangle_index,
                              Barycentric& bary) const;
    cv::Mat DrawTriangleLookup(cv::Mat triangle_lookup, int max) const;
    SpaceCoord ComputeCentroid() const;
    vector<vector<int> > GetVertexTriangleNeighbors() const;
    vector<vector<int> > GetTriangleTriangleNeighbors() const;
    cm AveragePointDistance(Pose a, Pose b) const;
    cm ApproximateDiameter() const;
    
    // Modify
    void AverageVertices(float strength, int iterations);
    void ConvertVertexColorsToTexture(int resolution);
    void ScaleVertices(float scale_factor);
    
    // Fill info on mesh object
    void SetUvNormalization(px rows, px cols, int texture_index);
    //void PopulateVertexFaceNeighbors();
    
    // Read/Write from file
    void ReadPly(string ply_path);
    void ReadObj(string obj_path);
    void WriteObj(string obj_path, string name) const;
    
    // Utility
    void DrawWireframe(
            cv::Mat& image, Pose pose, Intrinsics intrinsics,
            cv::Scalar color,
            bool backface_culling=false) const;
    
    void DrawConstant(
            cv::Mat& image,
            Pose pose,
            Intrinsics intrinsics,
            cv::Scalar color) const;
    
    void CreateUvProjection(vector<SpaceCoord> projections);

    void InitOpenGLRender();
    void DrawOpenGLModel(
        cv::Mat &image, Pose pose, Intrinsics intrinsics);
    void ReleaseOpenGLRender();
    bool isOpenGLOK();
};

class MeshSampler{
public:
    // Data members
    vector<px> area_cdf_;
    px max_prob_;
    uniform_real_variate* unit_variate_;
    uniform_real_variate* area_variate_;
    RNG_* rng_;
    Mesh* mesh_;
    vector<int> remap_;
    
    // Constructors/Destructors
    MeshSampler(RNG_*, Mesh*);
    MeshSampler(const MeshSampler&, boost::unordered_set<int>);
    ~MeshSampler();
    
    // Sampler methods
    int RandomTriangle();
    void RandomPointOnMesh(SpaceCoord&, ImCoord&, int&);
};
