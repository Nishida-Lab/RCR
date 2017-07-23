### robocar

version 0.0.0 in development


### dependency

- C++14 Standard Library
- Boost C++ Libraries 1.58.0
- OpenCV 3.2.0


### for_duration (include/robocar/chrono/for_duration.hpp)

テンプレート関数．一定時間ループを回したい時にどうぞ．<br>
第一引数に時間（`std::chrono::seconds`など），第二引数にファンクタを取ります．<br>
ファンクタには引数として経過時間と継続時間が渡されるので，
ジェネリックラムダを使うのが一番楽だと思います．<br>
引数が要らない場合はご自分で適当に類似品を作ってください．

使用例：

```
robocar::chrono::for_duration(std::chrono::seconds {5}, [](auto&& elapsed, auto&& duration)
{
  std::cout << "\r\e[K[debug] please wait for " << duration.count() - elapsed.count() << " sec" << std::flush;
  std::this_thread::sleep_for(std::chrono::seconds {1});
});
```

### runtime_typename (include/robocar/string/runtime_typename.hpp)

テンプレート関数．引数として与えられたオブジェクトの型名を`std::string`型のオブジェクトで返します．<br>
主にデバッグ用です．テンプレートメタプログラミング等で実行時の型名が知りたい時にどうぞ．<br>

