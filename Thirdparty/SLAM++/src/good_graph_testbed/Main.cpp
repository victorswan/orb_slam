/*
                                +-----------------------------------+
                                |                                   |
                                | ***  Simple BA iface example  *** |
                                |                                   |
                                |  Copyright  (c) -tHE SWINe- 2015  |
                                |                                   |
                                |             Main.cpp              |
                                |                                   |
                                +-----------------------------------+
*/

/**
 *	@file src/ba_interface_example/Main.cpp
 *	@brief contains the main() function of the simple example BA program
 *	@author -tHE SWINe-
 *	@date 2015-08-05
 */

#include <stdio.h> // printf
#include <fstream>
#include <string>
#include <ctime>        // std::time
#include <cstdlib>      // std::rand, std::srand
#include "good_graph_testbed/BAOptimizer.h" // BA types
// note that nothing else from SLAM++ needs to be included

#include "slam/Parser.h" // want to be able to parse .graph files
#include "slam_app/ParsePrimitives.h" // reuse SLAM++ standard parse primitives
// we need this to parse .graph files

int n_dummy_param = 0; // required by the DL solver, otherwise causes a link error

// perform 100-repeat test of lazier greedy vs. greedy baseline
//#define EVAL_LAZIER_GREEDY
// compare good graph with other subset selection methods
#define EVAL_SUBSET_METHODS


/**
 *	@brief a simple parse loop for BA systems
 */
class CMyParseLoop {
protected:
    CBAOptimizer &m_r_optimizer; /**< @brief reference to the optimizer */

public:
    /**
     *	@brief default constructor
     *	@param[in] r_optimizer is reference to the optimizer to be filled with edges and vertices
     */
    CMyParseLoop(CBAOptimizer &r_optimizer)
        :m_r_optimizer(r_optimizer)
    {}

    /**
     *	@brief appends the system with a general edge of unknown type
     *	@param[in] r_t_edge is the measurement to be appended
     */
#ifdef ENABLE_STEREO_SLAMPP
    // hack function to get around the compile error
    void AppendSystem(const CParserBase::TEdgeP2C3D &r_t_edge) // throw(std::bad_alloc)
    {
        //        m_r_optimizer.Add_P2SC3DEdge(r_t_edge.m_n_node_0, r_t_edge.m_n_node_1,
        //                                    r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma);
    }
#elif defined ENABLE_QUATERNION
    void AppendSystem(const CParserBase::TEdgeP2C3D_Quat &r_t_edge) // throw(std::bad_alloc)
    {
        m_r_optimizer.Add_P2C3DEdge(r_t_edge.m_n_node_0, r_t_edge.m_n_node_1,
                                    r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma);
    }
#else
    void AppendSystem(const CParserBase::TEdgeP2C3D &r_t_edge) // throw(std::bad_alloc)
    {
        m_r_optimizer.Add_P2C3DEdge(r_t_edge.m_n_node_0, r_t_edge.m_n_node_1,
                                    r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma);
        //
        map_pose_2_lmk[r_t_edge.m_n_node_1].push_back(r_t_edge.m_n_node_0);
    }
#endif


    /**
     *	@brief initializes a camera vertex
     *	@param[in] r_t_vertex is the vertex to be initialized
     */
#ifdef ENABLE_STEREO_SLAMPP
    // hack function to get around the compile error
    void InitializeVertex(const CParserBase::TVertexCam3D &r_t_vertex) // throw(std::bad_alloc)
    {
        //        m_r_optimizer.Add_SCamVertex(r_t_vertex.m_n_id, r_t_vertex.m_v_position);
        //
        //        poses_vtx_.push_back(r_t_vertex.m_n_id);
    }
#elif defined ENABLE_QUATERNION
    void InitializeVertex(const CParserBase::TVertexCam3D_Quat &r_t_vertex) // throw(std::bad_alloc)
    {
        m_r_optimizer.Add_CamVertex(r_t_vertex.m_n_id, r_t_vertex.m_v_position);
        //
        poses_vtx_.push_back(r_t_vertex.m_n_id);
    }
#else
    void InitializeVertex(const CParserBase::TVertexCam3D &r_t_vertex) // throw(std::bad_alloc)
    {
        m_r_optimizer.Add_CamVertex(r_t_vertex.m_n_id, r_t_vertex.m_v_position);
        //
        poses_vtx_.push_back(r_t_vertex.m_n_id);
    }
#endif


    /**
     *	@brief initializes a structure point vertex
     *	@param[in] r_t_vertex is the vertex to be initialized
     */
    void InitializeVertex(const CParserBase::TVertexXYZ &r_t_vertex) // throw(std::bad_alloc)
    {
        m_r_optimizer.Add_XYZVertex(r_t_vertex.m_n_id, r_t_vertex.m_v_position);
    }

    //
    std::vector<size_t> poses_vtx_;
    std::vector<size_t> reserv_pos_;
    std::map<size_t, std::vector<size_t>> map_pose_2_lmk;
};


//struct camStruct {
//    int id;
//    // c[0], c[1], c[2], axis(0), axis(1), axis(2), pKF->fx, pKF->fy, pKF->cx, pKF->cy, 0;
//    Eigen::Matrix<double, 11, 1> pose;
//    std::vector<int> lmk_ids;
//};

//struct lmkStruct {
//    int id;
//    //    double x, y, z;
//    Eigen::Vector3d pose;
//};

void splitString(const std::string & s,
                 const std::string & delimiter,
                 std::vector<std::string> &res) {
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    std::string token;
    res.clear();

    while ((pos_end = s.find (delimiter, pos_start)) != std::string::npos) {
        token = s.substr (pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back (token);
    }

    res.push_back (s.substr (pos_start));
}

void BuildSubGraph(const char * p_s_input_file, const std::vector<size_t> & vertex_subset,
                   CBAOptimizer & optimizer) {
    //
    size_t nidx = 0;
    // old id -> new id
    std::map<size_t, size_t> map_cam_subset, map_lmk_subset;
    for (const auto & ele : vertex_subset)
    {
        map_cam_subset.insert({ele, nidx});
        nidx ++;
    }

    std::ifstream fin;
    fin.open(p_s_input_file);
    if (fin.is_open()) {
        std::string line;
        while (std::getline(fin, line)) {
            //            std::cout << line << std::endl;
            //
            std::vector<std::string> words;
            splitString(line, std::string(" "), words);
            if (words.size() > 0) {
                if (words[0] == "VERTEX_CAM") {
                    assert(words.size() == 14);
                    size_t id = std::stoi(words[1]);
                    auto iter = map_cam_subset.find(id);
                    if (iter == map_cam_subset.end())
                        continue ;
                    // init rotation
                    Eigen::Quaternion<double> quat(
                                std::stod(words[8]),
                            std::stod(words[5]),
                            std::stod(words[6]),
                            std::stod(words[7]));
                    quat.normalize();
                    quat = quat.inverse();
                    //Eigen::Matrix3d Q = quat.toRotationMatrix();
                    //Q = Q.inverse().eval();
                    //Q = Q.householderQr().householderQ();
                    // init translation
                    Eigen::Vector3d t_vec(
                                std::stod(words[2]),
                            std::stod(words[3]),
                            std::stod(words[4]));
                    //rotate
                    Eigen::Vector3d c = quat * (-t_vec);
                    //Eigen::Vector3d axis = C3DJacobians::v_RotMatrix_to_AxisAngle(Q);
                    Eigen::Vector3d axis;
                    C3DJacobians::Quat_to_AxisAngle(quat, axis);
                    // assemble into eigen matrix
                    Eigen::Matrix<double, 11, 1> pose_camera_;
                    pose_camera_ << c[0], c[1], c[2], axis(0), axis(1), axis(2), std::stod(words[9]), std::stod(words[10]), std::stod(words[11]), std::stod(words[12]), std::stod(words[13]);
                    // insert to the flat system
                    //                    std::cout << "insert cam with id " << iter->second << std::endl;
                    optimizer.Add_CamVertex(iter->second, pose_camera_);
                }
                else if (words[0] == "VERTEX_XYZ") {
                    assert(words.size() == 5);
                    // do nothing at this point
                }
                else if (words[0] == "EDGE_PROJECT_P2MC") {
                    assert(words.size() == 8);

                    size_t lmk_id = std::stoi(words[1]), kf_id = std::stoi(words[2]);
                    if (map_cam_subset.find(kf_id) == map_cam_subset.end())
                        continue ;
                    if (map_lmk_subset.find(lmk_id) != map_lmk_subset.end())
                        continue ;
                    //                    std::cout << "insert lmk with id " << nidx << std::endl;
                    map_lmk_subset.insert({lmk_id, nidx});
                    nidx ++;
                }
                else {
                    fprintf(stderr, "error: no defined graph components from file\n");
                }
            }
        }
        fin.close();
    }
    // scan file again to fill in edges and lmks
    fin.open(p_s_input_file);
    if (fin.is_open()) {
        std::string line;
        while (std::getline(fin, line)) {
            //            std::cout << line << std::endl;
            //
            std::vector<std::string> words;
            splitString(line, std::string(" "), words);
            if (words.size() > 0) {
                if (words[0] == "VERTEX_CAM") {
                    // do nothing since already inserted
                }
                else if (words[0] == "VERTEX_XYZ") {
                    assert(words.size() == 5);
                    size_t id = std::stoi(words[1]);
                    auto iter = map_lmk_subset.find(id);
                    if (iter == map_lmk_subset.end())
                        continue ;
                    Eigen::Vector3d pose_lmk_;
                    pose_lmk_ << std::stod(words[2]), std::stod(words[3]), std::stod(words[4]);
                    optimizer.Add_XYZVertex(iter->second, pose_lmk_);
                }
                else if (words[0] == "EDGE_PROJECT_P2MC") {
                    assert(words.size() == 8);
                    size_t lmk_id = std::stoi(words[1]);
                    auto iter_1 = map_lmk_subset.find(lmk_id);
                    if (iter_1 == map_lmk_subset.end())
                        continue ;
                    size_t kf_id = std::stoi(words[2]);
                    auto iter_2 = map_cam_subset.find(kf_id);
                    if (iter_2 == map_cam_subset.end())
                        continue ;
                    Eigen::Vector2d v_observation;
                    v_observation << std::stod(words[3]), std::stod(words[4]);
                    Eigen::Matrix2d v_infomat;
                    v_infomat.setIdentity();
                    v_infomat(0, 0) = std::stod(words[5]);
                    v_infomat(0, 1) = std::stod(words[6]);
                    v_infomat(1, 1) = std::stod(words[7]);

                    optimizer.Add_P2C3DEdge(iter_1->second, iter_2->second, v_observation, v_infomat);
                }
                else {
                    fprintf(stderr, "error: no defined graph components from file\n");
                }
            }
        }
        fin.close();

    }
}


static bool compareWeightStruct(const std::pair<size_t, int> & p, const std::pair<size_t, int> & q) {
    return p.second > q.second;
}


/**
 *	@brief main
 *
 *	@param[in] n_arg_num is number of commandline arguments
 *	@param[in] p_arg_list is the list of commandline arguments
 *
 *	@return Returns 0 on success, -1 on failure.
 */
int main(int n_arg_num, const char **p_arg_list)
{
    bool b_verbose = true;
    const char *p_s_input_file = 0;
    for(int i = 1; i < n_arg_num; ++ i) {
        if(!strcmp(p_arg_list[i], "--help") || !strcmp(p_arg_list[i], "-h")) {
            printf("use: good_graph_testbed [-i|--input <input-graph-file>] [-q|--quiet]\n");
            return 0;
        } else if(!strcmp(p_arg_list[i], "--quiet") || !strcmp(p_arg_list[i], "-q"))
            b_verbose = false;
        else if(i + 1 == n_arg_num) {
            fprintf(stderr, "error: argument \'%s\': missing value or an unknown argument\n", p_arg_list[i]);
            return -1;
        } else if(!strcmp(p_arg_list[i], "--input") || !strcmp(p_arg_list[i], "-i"))
            p_s_input_file = p_arg_list[++ i];
        else {
            fprintf(stderr, "error: argument \'%s\': an unknown argument\n", p_arg_list[i]);
            return -1;
        }
    }
    if(!p_s_input_file) {
        fprintf(stderr, "error: no input file specified. run with -h or --help to get help\n");
        return -1;
    }
    // "parse" cmdline

#ifdef EVAL_LAZIER_GREEDY

    CBAOptimizer optimizer(b_verbose);
    // create the optimizer object

    //	{
#ifdef ENABLE_QUATERNION
    typedef MakeTypelist(CVertexXYZParsePrimitive, CVertexCam3DQuatParsePrimitive,
                         CEdgeP2C3DQuatParsePrimitive) CMyParsedPrimitives;
#else
    typedef MakeTypelist(CVertexXYZParsePrimitive, CVertexCam3DParsePrimitive,
                         CEdgeP2C3DParsePrimitive) CMyParsedPrimitives;
#endif
    typedef CParserTemplate<CMyParseLoop, CMyParsedPrimitives> CMyParser;
    CMyParseLoop parse_loop(optimizer);
    if(!CMyParser().Parse(p_s_input_file, parse_loop)) {
        fprintf(stderr, "error: failed to parse \'%s\'\n", p_s_input_file);
        return -1;
    }
    //	}
    // initialize the vertices, add edges

    size_t repeat_times = 100;
    size_t N = parse_loop.poses_vtx_.size(), M = N/2; // N-1; //
    printf("total num. of camera poses = %d\n", N);
    printf("target num. of camera poses = %d\n", M);
    //    optimizer.Dump_Graph("graph_orig.graph");

    // set the last camera state un-removable
    parse_loop.reserv_pos_.clear();
    parse_loop.reserv_pos_.push_back(0);
    parse_loop.reserv_pos_.push_back(1);

    // reduce the graph to meet cardinality constr. M
    double logdt_0 = optimizer.Find_Subgraph(M, 0, parse_loop.reserv_pos_);
    printf("reserv_vtx_.size() = %d; logDet(S) = %.03f\n", parse_loop.reserv_pos_.size(), logdt_0);
    printf("reserv_vtx_: ");
    for (size_t i=0; i<parse_loop.reserv_pos_.size(); ++i)
        printf("%d ", parse_loop.reserv_pos_[i]);
    printf("\n");
    optimizer.Show_Stats();

    double avg_diff_01 = 0, avg_diff_02 = 0;
    std::vector<size_t> diff_set, diff_01, diff_02;
    std::vector<size_t> subgraph_0 = parse_loop.reserv_pos_;
    std::sort(subgraph_0.begin(), subgraph_0.end());
    //    printf("index of selected nodes: ");
    //    for (size_t i=0; i<subgraph_0.size(); ++i)
    //        printf("%d ", subgraph_0[i]);
    //    printf("\n");

    optimizer.resetTime();
    for (size_t i = 0; i < repeat_times; ++i) {
        parse_loop.reserv_pos_.clear();
        parse_loop.reserv_pos_.push_back(0);
        parse_loop.reserv_pos_.push_back(1);
        double logdt_1 = optimizer.Find_Subgraph(M, 1, parse_loop.reserv_pos_);
        //        printf("reserv_vtx_.size() = %d; logDet(S) = %.03f\n", parse_loop.reserv_pos_.size(), logdt_1);
        //        printf("reserv_vtx_: ");
        //        for (size_t i=0; i<parse_loop.reserv_pos_.size(); ++i)
        //            printf("%d ", parse_loop.reserv_pos_[i]);
        //        printf("\n");
        //        optimizer.Show_Stats();

        std::vector<size_t> subgraph_1 = parse_loop.reserv_pos_;
        std::sort(subgraph_1.begin(), subgraph_1.end());
        diff_set.clear();
        std::set_difference(subgraph_0.begin(), subgraph_0.end(), subgraph_1.begin(), subgraph_1.end(),
                            std::inserter(diff_set, diff_set.begin()));
        diff_01.push_back(diff_set.size());
        avg_diff_01 += diff_set.size();
    }
    avg_diff_01 /= double(repeat_times);
    optimizer.Show_Stats();

    //    optimizer.resetTime();
    //    for (size_t i = 0; i < repeat_times; ++i) {
    //        parse_loop.reserv_pos_.clear();
    //        parse_loop.reserv_pos_.push_back(0);
    //        parse_loop.reserv_pos_.push_back(1);
    //        double logdt_2 = optimizer.Find_Subgraph(M, 2, parse_loop.reserv_pos_);
    //        //        printf("reserv_vtx_.size() = %d; logDet(S) = %.03f\n", parse_loop.reserv_pos_.size(), logdt_2);
    //        //        printf("reserv_vtx_: ");
    //        //        for (size_t i=0; i<parse_loop.reserv_pos_.size(); ++i)
    //        //            printf("%d ", parse_loop.reserv_pos_[i]);
    //        //        printf("\n");
    //        //        optimizer.Show_Stats();

    //        std::vector<size_t> subgraph_2 = parse_loop.reserv_pos_;
    //        std::sort(subgraph_2.begin(), subgraph_2.end());
    //        diff_set.clear();
    //        std::set_difference(subgraph_0.begin(), subgraph_0.end(), subgraph_2.begin(), subgraph_2.end(),
    //                            std::inserter(diff_set, diff_set.begin()));
    //        diff_02.push_back(diff_set.size());
    //        avg_diff_02 += diff_set.size();
    //    }
    //    avg_diff_02 /= double(repeat_times);
    //    optimizer.Show_Stats();

    printf("average diff: %.03f\n", avg_diff_01);
    for (size_t i=0; i<diff_01.size(); ++i)
        printf("%d ", diff_01[i]);
    printf("\n\n");

    //    printf("average diff: %.03f\n", avg_diff_02);
    //    for (size_t i=0; i<diff_02.size(); ++i)
    //        printf("%d ", diff_02[i]);
    //    printf("\n\n");

    // optimize the system
    //    optimizer.Optimize(20);
    //    optimizer.Show_Stats();

    //    // show the timing, save the results
    //    optimizer.Show_Stats();
    //    optimizer.Dump_Graph("graph_sub.graph");
    //    optimizer.Dump_State("solution.txt");

#elif defined EVAL_SUBSET_METHODS

    size_t N, M;
    double logdt;
    //
    bool do_good = true; // false; //
    bool do_covi = false; // true; //
    bool do_rand = false; // true; //
    // create the optimizer object
#ifdef ENABLE_QUATERNION
    typedef MakeTypelist(CVertexXYZParsePrimitive, CVertexCam3DQuatParsePrimitive,
                         CEdgeP2C3DQuatParsePrimitive) CMyParsedPrimitives;
#else
    typedef MakeTypelist(CVertexXYZParsePrimitive, CVertexCam3DParsePrimitive,
                         CEdgeP2C3DParsePrimitive) CMyParsedPrimitives;
#endif
    typedef CParserTemplate<CMyParseLoop, CMyParsedPrimitives> CMyParser;

    // stage 1: full graph optimization
    CBAOptimizer * optimizer_1 = new CBAOptimizer(b_verbose);
    CMyParseLoop parse_loop_1(*optimizer_1);
    if(!CMyParser().Parse(p_s_input_file, parse_loop_1)) {
        fprintf(stderr, "error: failed to parse \'%s\'\n", p_s_input_file);
        return -1;
    }

    N = parse_loop_1.poses_vtx_.size();
    M = N/2; // N/3; //

    // initialize the vertices, add edges
    optimizer_1->Dump_Graph("graph_raw.graph");
    //    optimizer_1.Plot3D("graph_raw.tga");

    printf("\n\n --------------- Start Full Graph Optimization ---------------\n\n");
    // optimize the system
    optimizer_1->Optimize(20);
    // show the timing, save the results
    optimizer_1->Show_Stats();
    optimizer_1->Dump_Graph("graph_fullOpt.graph");
    optimizer_1->Dump_State("solution_fullOpt.txt");
    //    optimizer_1.Plot3D("graph_fullOpt.tga");
    delete optimizer_1;

    if (do_good) {
        printf("\n\n --------------- Start Good Graph Optimization ---------------\n\n");
        // stage 2: good subgraph optimization
        CBAOptimizer * optimizer_2 = new CBAOptimizer(b_verbose);
        // create the optimizer object
        CMyParseLoop parse_loop_2(*optimizer_2);
        if(!CMyParser().Parse(p_s_input_file, parse_loop_2)) {
            fprintf(stderr, "error: failed to parse \'%s\'\n", p_s_input_file);
            return -1;
        }

        printf("total num. of camera poses = %d\n", N);
        printf("target num. of camera poses = %d\n", M);
        // set the last camera state un-removable
        parse_loop_2.reserv_pos_.clear();
        parse_loop_2.reserv_pos_.push_back(0);

        // reduce the graph to meet cardinality constr. M
        logdt = optimizer_2->Find_Subgraph(M, 1, parse_loop_2.reserv_pos_);
        sort(parse_loop_2.reserv_pos_.begin(), parse_loop_2.reserv_pos_.end());
        printf("reserv_vtx_.size() = %d; logDet(S) = %.03f\n", parse_loop_2.reserv_pos_.size(), logdt);
        printf("reserv_vtx_: ");
        for (size_t i=0; i<parse_loop_2.reserv_pos_.size(); ++i)
            printf("%d ", parse_loop_2.reserv_pos_[i]);
        printf("\n");
        optimizer_2->Show_Stats();

        // re-config the optimizer with selected subgraph
        CBAOptimizer * optimizer_2_sub = new CBAOptimizer(b_verbose);
        BuildSubGraph(p_s_input_file, parse_loop_2.reserv_pos_, *optimizer_2_sub);

        // optimize the system
        optimizer_2_sub->Optimize(20);
        // show the timing, save the results
        optimizer_2_sub->Show_Stats();
        optimizer_2_sub->Dump_Graph("graph_goodOpt.graph");
        optimizer_2_sub->Dump_State("solution_goodOpt.txt");
        //    optimizer_2_sub.Plot3D("graph_goodOpt.tga");
        delete optimizer_2;
        delete optimizer_2_sub;
    }

    if (do_covi) {
        printf("\n\n --------------- Start Co-vis Graph Optimization ---------------\n\n");
        // stage 3: co-vis weight
        CBAOptimizer * optimizer_3 = new CBAOptimizer(b_verbose);
        // create the optimizer object
        CMyParseLoop parse_loop_3(*optimizer_3);
        if(!CMyParser().Parse(p_s_input_file, parse_loop_3)) {
            fprintf(stderr, "error: failed to parse \'%s\'\n", p_s_input_file);
            return -1;
        }

        printf("total num. of camera poses = %d\n", N);
        printf("target num. of camera poses = %d\n", M);
        // set the last camera state un-removable
        parse_loop_3.reserv_pos_.clear();
        parse_loop_3.reserv_pos_.push_back(0);

        // reduce the graph to meet cardinality constr. M
        // find the shared portion of edges between parse_loop_3.reserv_pos_[0] and other cameras
        size_t root_cam = parse_loop_3.reserv_pos_[0];
        std::vector<std::pair<size_t, int>> weight_vec;
        std::vector<size_t> root_edge = parse_loop_3.map_pose_2_lmk[root_cam], diff_set;
        std::sort(root_edge.begin(), root_edge.end());
        for (size_t i=0; i<N; ++i) {
            if (parse_loop_3.poses_vtx_[i] == root_cam)
                continue ;
            std::vector<size_t> tmp_edge = parse_loop_3.map_pose_2_lmk[parse_loop_3.poses_vtx_[i]];
            std::sort(tmp_edge.begin(), tmp_edge.end());
            diff_set.clear();
            std::set_difference(root_edge.begin(), root_edge.end(), tmp_edge.begin(), tmp_edge.end(),
                                std::inserter(diff_set, diff_set.begin()));
            //
            int covis_weight = root_edge.size() - diff_set.size();
            weight_vec.push_back({i, covis_weight});
        }
        std::sort(weight_vec.begin(), weight_vec.end(), compareWeightStruct);
        //
        printf("co-vis weight: \n");
        for (size_t i=0; i<M; ++i) {
            parse_loop_3.reserv_pos_.push_back(weight_vec[i].first);
            printf("%d ", weight_vec[i].second);
        }
        printf("\n");
        sort(parse_loop_3.reserv_pos_.begin(), parse_loop_3.reserv_pos_.end());
        printf("reserv_vtx_.size() = %d; logDet(S) = %.03f\n", parse_loop_3.reserv_pos_.size(), logdt);
        printf("reserv_vtx_: ");
        for (size_t i=0; i<parse_loop_3.reserv_pos_.size(); ++i)
            printf("%d ", parse_loop_3.reserv_pos_[i]);
        printf("\n");
        optimizer_3->Show_Stats();

        // re-config the optimizer with selected subgraph
        CBAOptimizer * optimizer_3_sub = new CBAOptimizer(b_verbose);
        BuildSubGraph(p_s_input_file, parse_loop_3.reserv_pos_, *optimizer_3_sub);

        // optimize the system
        optimizer_3_sub->Optimize(20);
        // show the timing, save the results
        optimizer_3_sub->Show_Stats();
        optimizer_3_sub->Dump_Graph("graph_covisOpt.graph");
        optimizer_3_sub->Dump_State("solution_covisOpt.txt");
        delete optimizer_3;
        delete optimizer_3_sub;
    }

    if (do_rand) {
        printf("\n\n --------------- Start Random Graph Optimization ---------------\n\n");
        // stage 4: random
        CBAOptimizer * optimizer_4 = new CBAOptimizer(b_verbose);
        // create the optimizer object
        CMyParseLoop parse_loop_4(*optimizer_4);
        if(!CMyParser().Parse(p_s_input_file, parse_loop_4)) {
            fprintf(stderr, "error: failed to parse \'%s\'\n", p_s_input_file);
            return -1;
        }

        printf("total num. of camera poses = %d\n", N);
        printf("target num. of camera poses = %d\n", M);
        // set the last camera state un-removable
        parse_loop_4.reserv_pos_.clear();
        parse_loop_4.reserv_pos_.push_back(0);

        // reduce the graph to meet cardinality constr. M
        // std::srand( unsigned ( std::time(0) ) );
        std::random_shuffle (parse_loop_4.poses_vtx_.begin(), parse_loop_4.poses_vtx_.end());
        for (size_t i=0; i<M+1; ++i) {
            if (parse_loop_4.poses_vtx_[i] == parse_loop_4.reserv_pos_[0])
                continue ;
            parse_loop_4.reserv_pos_.push_back(parse_loop_4.poses_vtx_[i]);
            if (parse_loop_4.reserv_pos_.size() == M)
                break ;
        }
        //
        sort(parse_loop_4.reserv_pos_.begin(), parse_loop_4.reserv_pos_.end());
        printf("reserv_vtx_.size() = %d; logDet(S) = %.03f\n", parse_loop_4.reserv_pos_.size(), logdt);
        printf("reserv_vtx_: ");
        for (size_t i=0; i<parse_loop_4.reserv_pos_.size(); ++i)
            printf("%d ", parse_loop_4.reserv_pos_[i]);
        printf("\n");
        optimizer_4->Show_Stats();

        // re-config the optimizer with selected subgraph
        CBAOptimizer * optimizer_4_sub = new CBAOptimizer(b_verbose);
        BuildSubGraph(p_s_input_file, parse_loop_4.reserv_pos_, *optimizer_4_sub);

        // optimize the system
        optimizer_4_sub->Optimize(20);
        // show the timing, save the results
        optimizer_4_sub->Show_Stats();
        optimizer_4_sub->Dump_Graph("graph_randOpt.graph");
        optimizer_4_sub->Dump_State("solution_randOpt.txt");
        delete optimizer_4;
        delete optimizer_4_sub;
    }

#endif

    return 0;
}

/**
 *	@page baifaceexample Simple SLAM++ Interface Example
 *
 *	This example shows how to interface SLAM++ with other code, may that be C++ or even "C".
 *	It also demonstrates a very simple bundle adjustment (BA) optimizer, which takes graph files
 *	such as Venice (file venice871.g2o). The BA problem is posed using SE(3) global poses for
 *	cameras and global positions for points (rather than relative camera poses and points relative
 *	to camera).
 *
 *	The code is divided to two files, @ref Main.cpp and @ref BAOptimizer.cpp. All of SLAM++ heavy
 *	duty code is included only in BAOptimizer.cpp, while Main.cpp only includes the optimizer
 *	interface. This makes development of applications using SLAM++ easier, as Main.cpp now
 *	compiles in a few seconds (three seconds on a Core i7 laptop), while BAOptimizer.cpp takes
 *	a while to compile but needs to be rarely changed during the development.
 *
 *	We begin by creating an optimizer objecet:
 *
 *	@code
 *	CBAOptimizer optimizer
 *	@endcode
 *
 *	This object contains SLAM++ solver as well as the optimized system state. To fill it, one
 *	can call one of these functions:
 *
 * 	@code
 *	Eigen::Matrix<double, 11, 1> first_cam_pose, second_cam_pose;
 *	first_cam_pose << 0 0 0 0 0 0 500 500 0 0 0;
 *	optimizer.Add_CamVertex(0, first_cam_pose);
 *	second_cam_pose << 0 0 -10 0 0 0 500 500 0 0 0;
 *	optimizer.Add_CamVertex(1, second_cam_pose);
 *	// add a camera at the origin and another 10 units in front of it along z+
 *	// the optical centre is at (500, 500) pixels and the camera has no skew / distortion
 *	// (note that the poses are inverse - they transform points from world space to camera space)
 *
 *	optimizer.Add_XYZVertex(2, Eigen::Vector3d(10, 20, 30));
 *	// add a 3D point
 *
 *	optimizer.Add_P2C3DEdge(2, 0, Eigen::Vector2d(50, 50), Eigen::Matrix2d::Identity() * 10);
 *	optimizer.Add_P2C3DEdge(2, 1, Eigen::Vector2d(75, 75), Eigen::Matrix2d::Identity() * 10);
 *	// add an observation of the point by the two cameras, with covariance [10 0; 0 10]
 *	@endcode
 *
 *	We can access the state using:
 *
 * 	@code
 *	Eigen::Map<Eigen::VectorXd> camera_0_state = optimizer.r_Vertex_State(0);
 *	@endcode
 *
 *	Where the address of the vector data does not change over time (e.g. when more vertices are
 *	added, as would happen with std::vector elements).
 *	Now that we have the system ready, we can optimize using:
 *
 * 	@code
 *	optimizer.Optimize();
 *	@endcode
 *
 *	This also makes the value of camera_0_state change, so that the optimized camera pose
 *	is available. Alternately, we can save the results using:
 *
 * 	@code
 *	Dump_State("solution.txt");
 *	@endcode
 *
 *	It is also possible to save the optimized graph, like this:
 *
 * 	@code
 *	optimizer.Dump_Graph("solution.graph");
 *	@endcode
 *
 *	This is especially handy if doing processing from sensorial data - other researchers can
 *	then use this graph file without having to process the sensor data again. Note that this
 *	graph does not preserve the order in which the edges and vertices were added to the optimizer.
 *	It outputs all the vertices first (ordered by their indices), then outputs the edges (in the
 *	order they were introduced to the optimizer).
 *
 *	It is now possible to create a statically linked library which you can link to your existing
 *	code easily, and just use the simple optimizer interface where no templates are involved.
 *	@ref BAOptimizer.h also features a simple "C" interface which can be easily extended with
 *	additional functionality.
 *
 */

/*
 *	end-of-file
 */
