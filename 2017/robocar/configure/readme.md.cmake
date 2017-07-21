### ${PROJECT_NAME}

version ${PROJECT_VERSION} in development


### dependency

- C++14 Standard Library
- Boost C++ Libraries 1.58.0
- OpenCV 3.2.0


### robocar::chrono::for_duration

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

