## ESP-IDF 开发环境搭建

请参照ESP-IDF 入门指南： [https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/get-started/index.html](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/get-started/index.html)，按照步骤设置 ESP-IDF。

注意事项：
* 请完成链接页面的所有步骤。
* 请按照上面链接中的步骤构建一个或多个示例应用程序。

## ESP32(S2) 链接脚本修改

打开 ESP32(S2) 的链接脚本模板`${IDF_PATH}/components/esp32/ld/esp32.project.ld.in` 和 ` ${IDF_PATH}/components/esp32s2/ld/esp32s2.project.ld.in`, 将以下代码添加到 `.flash.rodata` 段的末尾.

```
   /* Parameters and log system datas */
    _param_start = .;
    KEEP(*(.param))
    KEEP(*(.param.*))
    _param_stop = .;
    . = ALIGN(4);
    _log_start = .;
    KEEP(*(.log))
    KEEP(*(.log.*))
    _log_stop = .;
    . = ALIGN(4);
```
以上代码可以实现，将具有 `.param.*` 或 `.log.*` 段属性的变量，放置在连续的存储区域，从而加快变量遍历速度。