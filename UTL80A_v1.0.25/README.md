# UTL80A · v1.0.25

XIAO nRF54L15 韌體與瀏覽器監視器。使用八路 LDC1614 讀值估算六軸力與力矩，支援 USB 與 BLE。串流目標為 100 Hz，網頁保留最近 2,000 筆。

## 目前這版怎麼運作

- **訊號與歸零。** 韌體收集八路有效 RAW28，扣除裝機後的無載零點，再在 MCU 上換算。開機自動歸零，換治具後可在網頁手動歸零。力量單位為 N，力矩為 N·m。
- **預設六軸 TCN。** 6×8 線性矩陣換算當筆資料，再加上因果 TCN 從當筆與過去 120 ms 算出的修正。歷史未齊時先輸出線性估計。可用「本版線性基準」比較同一組線性係數。
- **Fz 砝碼測試。** 這是可選的垂直砝碼對照模式，只替換 Fz 的換算，其他五軸沿用 TCN。它使用 0.5、1、1.5 kg 的實測步進與 `Fz_0` 的 300 秒空載資料。ANSYS 匯入保留一個載入案例的六軸施加值與八路響應。
- **目前的驗證範圍。** 兩批 1 kg 的 Fz 回放結果有明顯差異，Fz 砝碼模式在扭 Tz 時也會放大 Fz 讀值。開機使用原六軸 TCN，實機有效 100 Hz、0.1 N 解析度與 ±1% FS 準確度仍待整機驗證。查看 [Fz 數值](reports/fz_validation_v1.0.25.json) 或 [TCN 公式及比較](docs/TCN_校正與線性驗證.md)。

## 上傳到 Arduino

1. 在 Arduino IDE 的附加開發板管理員網址加入 `https://raw.githubusercontent.com/lolren/nrf54-arduino-core/main/package_nrf54l15clean_index.json`。
2. 安裝 nRF54L15 Boards 核心 1.0.23，選擇 XIAO nRF54L15。
3. 開啟 [UTL80A_v1.0.25.ino](arduino/UTL80A_v1.0.25/UTL80A_v1.0.25.ino) 並上傳。整個 sketch 資料夾內的標頭跟主程式一起使用。[Arduino sketch 命名規格](https://docs.arduino.cc/arduino-cli/sketch-specification/)

## 開啟監視器

Windows 執行 [Start-UTL80A.bat](Start-UTL80A.bat)，用 Chrome 或 Edge 開啟監視器。裝好治具並卸載後，按「手動歸零」。預設為 TCN，垂直砝碼量測可選「Fz 砝碼測試」。詳見 [使用說明](docs/使用說明.md)。

## 專案結構

| 位置 | 內容 |
| --- | --- |
| [arduino/](arduino/) | Arduino 主程式、全部標頭與 Serial Plotter 工具 |
| [web/](web/) | 監視器 HTML 與 JavaScript |
| [calibration/](calibration/) | Fz 係數重建及 TCN 訓練程式 |
| [reports/](reports/) | 有版號的 Fz 與 TCN 驗證結果 |
| [docs/](docs/) | 使用說明、公式與校正驗證 |

## 重建係數

韌體已附本版係數，可直接編譯。要重建時，在專案根目錄建立 `Tool`，切換到 `Tool` 並準備 [Python 套件](calibration/requirements.txt)。提供自己的原始工作簿資料夾 `TestData_1116` 與校正參考 `experiments.csv`。

```powershell
python -m pip install --target .\python -r ..\calibration\requirements.txt
$env:PYTHONPATH = (Resolve-Path .\python).Path
python ..\calibration\fit_fz.py --data-dir "D:\Measurements\TestData_1116" --reference-csv "D:\Measurements\experiments.csv"
python ..\calibration\train_tcn.py --data-dir "D:\Measurements\TestData_1116" --reference-csv "D:\Measurements\experiments.csv"
```

重建結果會寫回 `arduino/` 的係數標頭與 `reports/` 的驗證檔。新增 ANSYS 案例時，可在 Fz 命令加上 `--simulation` 和 CSV 路徑。欄位格式見 [ANSYS 匯入說明](docs/使用說明.md#下一批-ansys-資料怎麼接)。

## 發布到 GitHub

把本資料夾的**內容**放到 GitHub repository 根目錄，讓 GitHub 首頁直接顯示這份 README。第一次提交可用 `Release v1.0.25`，發布時建立 `v1.0.25` tag。原始量測、歷史封裝與本機工具留在專案工作區。
