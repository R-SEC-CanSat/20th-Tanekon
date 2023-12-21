#UnitVで色検出
import sensor
import image

sensor.reset()                      # カメラの初期化
sensor.set_pixformat(sensor.RGB565) # RGB565フォーマットを使用
sensor.set_framesize(sensor.QVGA)   # QVGAフレームサイズを使用(320*240)
sensor.set_auto_gain(False)         # オートゲインをオフ
sensor.__write_reg(0x80, 0xEF)      # オートホワイトバランスをオフ
sensor.skip_frames(time = 2000)     # 2秒間フレームをスキップして安定化

threshold = [(30, 100,  15, 127,  15, 127)] # 検出する色のしきい値を設定、閾値エディタを使用して取得

while(True):
    img = sensor.snapshot()           # 画像を取得
    blobs = img.find_blobs(threshold) # しきい値内の色を検出
    if blobs:
        max_blob = max(blobs, key=lambda b: b.area()) # 面積が最大の領域を取得
        img.draw_rectangle(max_blob[0:4])             # 検出した色を矩形で囲む
        img.draw_cross(max_blob[5], max_blob[6])      # 検出した色の中心に十字を描く
