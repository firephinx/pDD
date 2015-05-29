#pragma once
#include "settings.h"
#include "units.h"
#include "mesh.h"
#include "slic.h"
#include "binomial.h"
#include <vector>

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "yaml-cpp/yaml.h"

#define DEFAULT_NORMAL_THRESHOLD -0.35

/*
The Model class contains the information necessary to recognize a particular
object.
*/

using namespace std;

class Model{
public:
    struct Edge{
    public:
        int a;
        int b;
        int color_difference;
    };

//private:
    vector< vector< SpaceCoord > > patches_;
    vector< vector< image_color > > patch_colors_;
    vector< vector< Edge > > edges_;
    vector< vector< SpaceCoord > > normals_;
    vector< px > patch_radii_;
    SpaceCoord centroid_;
    
    // Debug Info.  These members will only be stored if the 'store_debug_info'
    // option of the constructor is set to true.
    vector< vector< ImCoord > > patches_uv_;
    vector< vector< int > > triangle_indices_;
    vector< px > px_per_cm_;
    vector< vector< int > > texture_assignment_;
    vector< cv::Mat > textures_;
    vector< cv::Mat > masks_;
    vector< vector< cv::Mat > > segmentations_;
    vector< vector< vector< SlicPoint > > > slic_centers_;
    vector< vector< int > > patch_superpixels_;
    vector< vector< int > > superpixel_patches_;
    Mesh construction_mesh_;

public:
    Model();
    
    Model(string file_path, bool load_debug_data=true);
    
    Model(Mesh& mesh,
          vector<cv::Mat>& textures,
          //const vector<int>& num_super_pixels,
          const float& superpixel_compactness,
          const vector<cm>& billboard_radii,
          const float& billboard_density,
          const float& edge_radius,
          const bool& store_debug_info=false);
    
    void GeneratePatchPositions(int layer_index,
                                int texture_index,
                                const Mesh& mesh,
                                const cv::Mat& integral_mask,
                                const cv::Mat& integral_texture,
                                const vector<SlicPoint>& centers,
                                const cv::Mat& triangle_lookup,
                                const vector<px>& px_per_cm,
                                vector<int>& patch_superpixels,
                                vector<int>& superpixel_patches,
                                vector<int>& triangles,
                                bool store_debug_info);
    
    Model(Mesh& mesh,
          vector<cv::Mat>& textures,
          const int& num_features,
          const px& max_spread,
          const px& min_spread,
          const float& min_color_dist,
          const vector<cm>& patch_radii,
          const vector<int>& patch_counts,
          const int& refinement_iterations,
          const bool& debug);
    
    void GenerateTextureFeatures(const Mesh&, const vector<cv::Mat>&,
            const vector<cv::Mat>&, const int&);
    
    void GeneratePatchesFromTexture(const Mesh& mesh,
                                    vector<cv::Mat>& textures,
                                    const vector<cv::Mat>& integral_textures,
                                    const vector<cv::Mat>& integral_masks,
                                    MeshSampler& mesh_sampler,
                                    const vector<px>& patch_radii,
                                    const vector<int>& patch_counts,
                                    const px& max_spread,
                                    const px& min_spread,
                                    const float& min_color_dist,
                                    int refinement_iterations,
                                    const bool& debug);
    
    bool FindNeighboringPatch(const cv::Mat& integral_texture,
                              const cv::Mat& integral_mask,
                              MeshSampler& mesh_sampler,
                              const ImCoord& point_a,
                              const integral_color& color_a,
                              const int& retries,
                              const float& min_color_dist,
                              const px& min_spread,
                              const px& max_spread,
                              const px& patch_radius_px,
                              SpaceCoord& world_b,
                              ImCoord& point_b,
                              int& triangle_b,
                              integral_color& color_b,
                              float& color_dist);
    
    //int GetOriginalPatchColors(vector<cv::Mat> integral_textures)
    
    int DrawTextureColorInfo(vector<cv::Mat>& textures,
                             vector<cv::Mat>& results,
                             const int& layer,
                             const bool& draw_text,
                             const cv::Scalar& bg_color,
                             const cv::Scalar& text_color) const;
    
    int DrawTexturePatches(vector<cv::Mat>& textures,
                           const int& layer,
                           const bool& draw_patches,
                           const bool& draw_patch_indices,
                           const bool& draw_edges,
                           const bool& draw_triangles,
                           const cv::Scalar& patch_color,
                           const cv::Scalar& patch_index_color,
                           const cv::Scalar& edge_color,
                           const cv::Scalar& triangle_color) const;
    
    int GenerateDebugInfo(string filepath);
    
    void GenerateBorderSamples(const Mesh&, const vector<int>&,
            MeshSampler&, const vector<px>&);
    
    vector<BinomialSample> Detect(const vector<Pose>& poses,
                                  const cv::Mat& integral_image,
                                  const Intrinsics& intrinsics,
                                  const int& layer) const;
    BinomialSample DetectRelativeDepth(const Pose& pose,
                                       const cv::Mat& depth_image,
                                       const Intrinsics& intrinsics,
                                       const int& layer) const;
    BinomialSample DetectDepth(const Pose& pose,
                               const cv::Mat& depth_image,
                               const Intrinsics& intrinsics,
                               const int& layer,
                               const cm& depth_tolerance) const;
    BinomialSample DetectColor(const Pose& pose,
                               const cv::Mat& integral_image,
                               const Intrinsics& intrinsics,
                               const int& layer) const;
    BinomialSample DetectContour(const Pose& pose,
                                 const cv::Mat& integral_image,
                                 const cv::Mat& depth_image,
                                 const Intrinsics& intrinsics,
                                 const int& layer,
                                 const int& difference_threshold,
                                 cv::Mat& debug_image) const;
    BinomialSample Detect(const Pose& pose,
                          const cv::Mat& integral_image,
                          const Intrinsics& intrinsics,
                          const int& layer,
                          vector<int>* edge_responses=NULL) const;
    
    vector<int> DetectAndDraw(Mesh mesh,
                              Pose base_pose,
                              vector<Pose> poses,
                              cv::Mat image,
                              Intrinsics intrinsics,
                              int layer,
                              BinomialInferenceModel inference_model,
                              //vector<double>& probs,
                              vector<int>& correct,
                              vector<int>& incorrect,
                              vector<int>& visible_poses);
    
    Pose AdaptiveRefine(cv::Mat integral_image,
                        Intrinsics intrinsics,
                        int start_layer,
                        Pose pose,
                        SpaceCoord pivot,
                        BinomialInferenceModel inference_model,
                        vector<vector< Pose > > layer_offsets,
                        int branch_factor=2);
    
    vector<vector<float> > GenerateViewHistograms(int layer,
                                                  vector<Pose> poses,
                                                  int bins);
    
    void Write(string out_path, bool store_debug_data=false);
    void Read(string in_path, bool load_debug_data=true);
    void ReadYamlNode(YAML::Node& node, bool load_debug_data=true);
    
    bool HasDebugInfo() const;
    bool IsVisible(const SpaceCoord& position,
                   const SpaceCoord& normal,
                   float normal_threshold=DEFAULT_NORMAL_THRESHOLD) const;
    void DrawPose(cv::Mat&,
                  const Intrinsics&,
                  const Pose&,
                  int layer,
                  bool draw_patches=true,
                  bool draw_edges=true,
                  bool draw_normals=false,
                  bool use_original_colors=false,
                  vector<int>* edge_responses=NULL);
    
    //void DrawTextureDebug(cv::Mat&, const Mesh&);
};

#ifdef USE_CUDA
class GpuModel{
public:
    GpuModel();
    GpuModel(Model);
    ~GpuModel();
    
    int* layer_indices_;
    float* positions_;
};
#endif

void operator >> (const YAML::Node&, vector< Model::Edge >&);
void operator >> (const YAML::Node&, vector< vector < Model::Edge > >&);
