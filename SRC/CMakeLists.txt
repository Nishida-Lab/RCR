cmake_minimum_required (VERSION 2.6)

#---------------------------------------------------
# コンパイル時のオプションの設定
#---------------------------------------------------
# リリースビルド用オプション
set(CMAKE_CPP_FLAGS_TELEASE "-Wall -O2")
# デバッグビルド用オプション
set(CMAKE_CPP_FLAGS_DEBUG "-g -O0")
# ビルドタイプ（下記の場合、Release）
set(CMAKE_BUILD_TYPE Release)
# 出力先の設定
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../) 
#----------------------------------------------------

#----------------------------------------------------
# ライブラリの追加
#----------------------------------------------------
# # OpenCV Library
# find_package(OpenCV REQUIRED)

# set( RCR2015_DIR
#   /home/pi/RCR2015
# )

# # Raspberry Pi OpenCV --------------- 
# # インクルードディレクトリ
# include_directories(
#   ${RCR2015_DIR}/LIB/OpenCV/robidouille/raspicam_cv
# )
# # ライブラリディレクトリ
# link_directories(
#   ${RCR2015_DIR}/LIB/OpenCV/robidoulle/raspicam_cv
#   ${RCR2015_DIR}/LIB/OpenCV/userland/build/lib
# )
# #-------------------------------------

# # ライブラリ郡の設定
# set( RASPI_OPENCV_LIBS
#   raspicamcv 
#   mmal_core 
#   mmal 
#   mmal_util 
#   vcos 
#   bcm_host
# )

#-----------------------------------------------------

include_directories(
  /usr/local/include
)
# ライブラリディレクトリ
link_directories(
  /usr/local/lib
)


# プロジェクト名
project(RCR)
# 静的リンクライブラリの追加
# add_subdirectory(Camera)
add_subdirectory(Motor)

# Mainビルド
add_executable (main main.cpp)

# ターゲットリンクライブラリ
target_link_libraries(main Motor wiringPi pthread boost_thread m)

