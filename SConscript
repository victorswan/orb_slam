#!/usr/bin/python

Import('coralEnv')

#=================================================
#DBoW2
DBoW2 = coralEnv.SharedLibrary(
    target = 'DBoW2_gf_orb_slam2',
    source = [
            'Thirdparty/DBoW2/DBoW2/BowVector.cpp',
	    'Thirdparty/DBoW2/DBoW2/FeatureVector.cpp',
	    'Thirdparty/DBoW2/DBoW2/FORB.cpp',
	    'Thirdparty/DBoW2/DBoW2/ScoringObject.cpp',
	    'Thirdparty/DBoW2/DUtils/Random.cpp',
	    'Thirdparty/DBoW2/DUtils/Timestamp.cpp',
	],
    CPPPATH = coralEnv['CPPPATH'] + ['Thirdparty/DBoW2/DBoW2', 'Thirdparty/DBoW2/DUtils'],
    )

coralEnv.InstallLibrary( [DBoW2] )

#=================================================
#g2o
g2o = coralEnv.SharedLibrary(
    target = 'g2o_gf_orb_slam2',
    source = [
            #types
	    'Thirdparty/g2o/g2o/types/types_sba.cpp',
	    'Thirdparty/g2o/g2o/types/types_six_dof_expmap.cpp',
	    'Thirdparty/g2o/g2o/types/types_seven_dof_expmap.cpp',
	    #core
	    'Thirdparty/g2o/g2o/core/hyper_graph_action.cpp',
	    'Thirdparty/g2o/g2o/core/hyper_graph.cpp',
	    'Thirdparty/g2o/g2o/core/marginal_covariance_cholesky.cpp',
	    'Thirdparty/g2o/g2o/core/matrix_structure.cpp',
	    'Thirdparty/g2o/g2o/core/batch_stats.cpp',
	    'Thirdparty/g2o/g2o/core/parameter.cpp',
	    'Thirdparty/g2o/g2o/core/cache.cpp',
	    'Thirdparty/g2o/g2o/core/optimizable_graph.cpp',
	    'Thirdparty/g2o/g2o/core/solver.cpp',
	    'Thirdparty/g2o/g2o/core/optimization_algorithm_factory.cpp',
	    'Thirdparty/g2o/g2o/core/estimate_propagator.cpp',
	    'Thirdparty/g2o/g2o/core/factory.cpp',
	    'Thirdparty/g2o/g2o/core/sparse_optimizer.cpp',
	    'Thirdparty/g2o/g2o/core/hyper_dijkstra.cpp',
	    'Thirdparty/g2o/g2o/core/parameter_container.cpp',
	    'Thirdparty/g2o/g2o/core/optimization_algorithm.cpp',
	    'Thirdparty/g2o/g2o/core/optimization_algorithm_with_hessian.cpp',
	    'Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.cpp',
	    'Thirdparty/g2o/g2o/core/jacobian_workspace.cpp',
	    'Thirdparty/g2o/g2o/core/robust_kernel.cpp',
	    'Thirdparty/g2o/g2o/core/robust_kernel_factory.cpp',
	    'Thirdparty/g2o/g2o/core/robust_kernel_impl.cpp',
	    #stuff
	    'Thirdparty/g2o/g2o/stuff/timeutil.cpp',
	    'Thirdparty/g2o/g2o/stuff/os_specific.c',
	    'Thirdparty/g2o/g2o/stuff/string_tools.cpp',
	    'Thirdparty/g2o/g2o/stuff/property.cpp',
	],
    CPPPATH = coralEnv['CPPPATH'] + ['Thirdparty/g2o/g2o/core', 'Thirdparty/g2o/g2o/stuff', 'Thirdparty/g2o/g2o/types'],
    )

coralEnv.InstallLibrary( [g2o] )

#=================================================
#ORB_SLAM2

coralEnv.Tool('cuda')
#coralEnv.Append(LIBS=['cutil', 'glut', 'GLEW'])

coralEnv['NVCCFLAGS'] = "-std=c++11",

GF_ORB_SLAM2 = coralEnv.SharedLibrary(
    target = 'GF_ORB_SLAM2',
    source = [
#            "include/Util_cuda.hpp",
#	    "include/Util.hpp",
	    "src/System.cc",
	    "src/Observability.cc",
	    "src/Tracking.cc",
	    "src/LocalMapping.cc",
	    "src/LoopClosing.cc",
	    "src/ORBextractor.cc",
	    "src/ORBmatcher.cc",
	    "src/FrameDrawer.cc",
	    "src/Converter.cc",
	    "src/MapPoint.cc",
	    "src/KeyFrame.cc",
	    "src/Map.cc",
	    "src/MapDrawer.cc",
	    "src/Optimizer.cc",
	    "src/PnPsolver.cc",
	    "src/Frame.cc",
	    "src/KeyFrameDatabase.cc",
	    "src/Sim3Solver.cc",
	    "src/Initializer.cc",
	    "src/Viewer.cc",
#	    "src/FAST_NEON.cc",
            "src/cuda/Allocator_gpu.cu",
	    "src/cuda/Fast_gpu.cu",
	    "src/cuda/Orb_gpu.cu",
	    "src/cuda/Cuda.cu",
	    "Thirdparty/SLAM++/src/good_graph_testbed/BAOptimizer.cpp",
	],
    LIBS = coralEnv['LIBS'] + ['g2o_gf_orb_slam2', 'DBoW2_gf_orb_slam2', 'pangolin', 'openblas', 'armadillo', 'gtest'],
    CPPPATH = ['Thirdparty/DBoW2/DBoW2', 'Thirdparty/DBoW2/DUtils', 'Thirdparty/SLAM++/include']
              + ['Thirdparty/g2o/g2o/core', 'Thirdparty/g2o/g2o/stuff', 'Thirdparty/g2o/g2o/types', 'Thirdparty/g2o/g2o/solvers']
	      + ['include', 'gf_orb_slam2', '.']
	      + ['/opt/armadillo/include']
	      + coralEnv['CPPPATH'],
    LIBPATH = ['/opt/armadillo/lib'] + coralEnv['LIBPATH'],
    NVCCINC = ["-I src/coral/libs/gf_orb_slam2/include", "-I /opt/opencv/include"] + coralEnv['NVCCINC'],
    CCFLAGS = ['-std=c++11', '-fPIC'] + coralEnv['CCFLAGS'],
    )

coralEnv.InstallLibrary( [GF_ORB_SLAM2] )
