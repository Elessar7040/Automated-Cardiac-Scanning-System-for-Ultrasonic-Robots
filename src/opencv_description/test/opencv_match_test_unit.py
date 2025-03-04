import cv2


def find_most_similar_region(image_a_path, image_b_path, scale_factors):
    # 读取图片
    img_a = cv2.imread(image_a_path)
    img_b = cv2.imread(image_b_path)

    # 转为灰度图
    gray_a = cv2.cvtColor(img_a, cv2.COLOR_BGR2GRAY)
    gray_b = cv2.cvtColor(img_b, cv2.COLOR_BGR2GRAY)

    best_match = None
    best_score = -100000000
    best_rect = None

    # 遍历不同缩放比例
    for scale in scale_factors:
        scaled_b = cv2.resize(gray_b, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
        if scaled_b.shape[0] >= gray_a.shape[0] or scaled_b.shape[1] >= gray_a.shape[1]:
            break
        res = cv2.matchTemplate(gray_a, scaled_b, cv2.TM_CCOEFF_NORMED)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

        if max_val > best_score:
            best_score = max_val
            best_match = scaled_b
            best_rect = (max_loc[0], max_loc[1], scaled_b.shape[1], scaled_b.shape[0])

    # 在图片A上绘制矩形
    top_left = (best_rect[0], best_rect[1])
    bottom_right = (best_rect[0] + best_rect[2], best_rect[1] + best_rect[3])
    result_img = img_a.copy()
    cv2.rectangle(result_img, top_left, bottom_right, (0, 255, 0), 2)

    return result_img, best_score

# 参数设置
# image_a_path = "src/opencv_description/opencv_description/POPY_background.png"
# image_b_path = "src/opencv_description/opencv_description/POPY_profile.png"
image_a_path = "src/opencv_description/opencv_description/img1.jpg"
image_b_path = "src/opencv_description/opencv_description/template1.png"
scale_factors = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4]  # 尝试的缩放比例

# 执行代码
result_img, best_score = find_most_similar_region(image_a_path, image_b_path, scale_factors)

# 显示结果
print(f"Best match score: {best_score}")
cv2.imshow("Result", result_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
