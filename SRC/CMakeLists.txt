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
#----------------------------------------------------

#----------------------------------------------------
# ライブラリの追加
#----------------------------------------------------
# OpenCV Library
find_package(OpenCV REQUIRED)

# Raspberry Pi OpenCV --------------- 
# インクルードディレクトリ
include_directories(
  /home/pi/RCR/LIB/OpenCV/robidouille/raspicam_cv
)
# ライブラリディレクトリ
link_directories(
  /home/pi/RCR/LIB/OpenCV/robidouille/raspicam_cv
  /home/pi/RCR/LIB/OpenCV/userland/build/lib
)
#-------------------------------------

# ライブラリ郡の設定
set( RASPI_OPENCV_LIBS
  raspicamcv 
  mmal_core 
  mmal 
  mmal_util 
  vcos 
  bcm_host
)

#-----------------------------------------------------


# プロジェクト名
project(RCR)
# 静的リンクライブラリの追加
add_subdirectory(Camera)

# Mainビルド
add_executable (main main.cpp)
# ターゲットリンクライブラリ
target_link_libraries(main Camera ${OpenCV_LIBS} ${RASPI_OPENCV_LIBS} wiringPi pthread m)

