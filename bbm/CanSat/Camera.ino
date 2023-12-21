import sensor
import image

sensor.reset()                      # カメラの初期化
sensor.set_pixformat(sensor.RGB565) # RGB565フォーマットを使用
sensor.set_framesize(sensor.QVGA)   # QVGAフレームサイズを使用
sensor.skip_frames(time = 2000)     # 2秒間フレームをスキップして安定化

threshold = [(30, 100,  15, 127,  15, 127)] # 検出する色のしきい値を設定

while(True):
    img = sensor.snapshot() # 画像を取得
    blobs = img.find_blobs(threshold) # しきい値内の色を検出
    if blobs:
        for b in blobs:
            img.draw_rectangle(b[0:4]) # 検出した色を矩形で囲む
            img.draw_cross(b[5], b[6]) # 検出した色の中心に十字を描く
