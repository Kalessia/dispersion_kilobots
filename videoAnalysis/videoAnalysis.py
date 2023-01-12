import time
import cv2
import os
import math
import shutil
import json


def video_to_images(name_video, name_file=None, frames_per_second=1):
    print("----------------------------------------------------")
    print("VIDEO : " + name_video.upper())
    print("----------------------------------------------------")

    if name_file is None:
        path_video = "./data/videos/" + name_video + ".mp4"
        frame_path = './data/frames/' + name_video
    else:
        path_video = "./data/videos/" + name_file + "/" + name_video + ".mp4"
        frame_path = "./data/frames/" + name_file + "/" + name_video

    try:
        # creating a folder named data
        if not os.path.exists("data"):
            os.makedirs("data")
        if not os.path.exists("./data/frames"):
            os.makedirs("./data/frames")
        if name_file is not None:
            if not os.path.exists("./data/frames/" + name_file):
                os.makedirs("./data/frames/" + name_file)
        if not os.path.exists(frame_path):
            os.makedirs(frame_path)
        else:
            shutil.rmtree(frame_path)
            os.mkdir(frame_path)
    except OSError:  # if not created then raise error
        print('Error: Creating directory of data')

    # Read the video from specified path
    cam = cv2.VideoCapture(path_video)

    frame_rate = cam.get(cv2.CAP_PROP_FPS)  # video frame rate

    # frame
    current_frame = 0
    num_frame = 0

    if frames_per_second > frame_rate or frames_per_second == -1:
        frames_per_second = frame_rate

    while True:
        # reading from frame
        ret, frame = cam.read()

        if ret:
            if current_frame % (math.floor(frame_rate / frames_per_second)) == 0:
                # if video is still left continue creating images
                if len(str(num_frame)) == 1:
                    num_frame_str = "000" + str(num_frame)
                elif len(str(num_frame)) == 2:
                    num_frame_str = "00" + str(num_frame)
                elif len(str(num_frame)) == 3:
                    num_frame_str = "0" + str(num_frame)
                else:
                    num_frame_str = str(num_frame)

                name = frame_path + '/' + name_video + '_frame' + num_frame_str + '.jpg'
                # print('Creating...' + name)

                # writing selected frames to images_path
                cv2.imwrite(name, frame)

                num_frame += 1

            current_frame += 1
        else:
            break

    # Release all space and windows once done
    cam.release()
    cv2.destroyAllWindows()

    return frame_path


def duration_video(name_video):
    # Load the video
    cap = cv2.VideoCapture("./data/videos/" + name_video + ".mp4")

    # Get the number of frames and the frame rate
    num_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS)

    # Calculate the duration of the video in seconds
    duration = num_frames / fps

    cap.release()

    return duration


def detect_black_edges(name_video, name_image, path_image=None):
    # Read image
    path_data_image = "./data/images/"
    if path_image is None:
        path = path_data_image + name_image + ".jpg"
    else:
        path = path_image + "/" + name_image + ".jpg"
    img = cv2.imread(path)

    # convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # threshold
    thresh = cv2.threshold(gray, 30, 255, cv2.THRESH_BINARY)[1]

    # write result to disk
    new_name = name_image + "_be"
    path_dir = path_data_image + name_video + "/be"
    path_save = path_dir + "/" + new_name + ".jpg"
    cv2.imwrite(path_save, thresh)
    # print("Image with black edges detection save to : " + path_save)

    cv2.destroyAllWindows()

    return new_name, path_dir


def remove_shadow(name_video, name_image, path_image=None):
    # Read image
    path_data_image = "./data/images/"
    if path_image is None:
        path = path_data_image + name_image + ".jpg"
    else:
        path = path_image + "/" + name_image + ".jpg"
    img = cv2.imread(path)

    # Convert the image to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Equalize the histogram
    enhanced = cv2.equalizeHist(gray)

    # Save the result
    new_name = name_image + "_rs"
    path_dir = path_data_image + name_video + "/rs"
    path_save = path_dir + "/" + new_name + ".jpg"
    cv2.imwrite(path_save, enhanced)
    # print("Image with shadows removed save to : " + path_save)

    cv2.destroyAllWindows()

    return new_name, path_dir


def detect_round_objects(name_video, name_image, p1, p2, nb_kilobots=-1, path_image=None, obj_type="kilobots"):
    # Read image
    path_data_image = "./data/images/"
    if path_image is None:
        path = path_data_image + name_image + ".jpg"
    else:
        path = path_image + "/" + name_image + ".jpg"
    img = cv2.imread(path)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Use the HoughCircles function to detect circles
    if obj_type == "disk":
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20, param1=p1, param2=p2, minRadius=100, maxRadius=140)
    elif obj_type == "ring":
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20, param1=p1, param2=p2, minRadius=160, maxRadius=200)
    else:  # detection de kilobots par défaut
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20, param1=p1, param2=p2, minRadius=8, maxRadius=15)

    params = None
    # Loop over the circles and draw them on the image
    if circles is not None:
        if (obj_type == "kilobots" and len(circles[0]) == nb_kilobots) or (obj_type != "kilobots" and len(circles[0]) == 1):
            params = (p1, p2)

            for (x, y, radius) in circles[0]:
                cv2.circle(img, (int(x), int(y)), int(radius), (0, 255, 0), 2)

            # save result
            if obj_type == "kilobots":
                new_name = name_image + "_kb_" + str(p1) + "_" + str(p2)
                path_dir = path_data_image + name_video + "/kb"
                path_save = path_dir + "/" + new_name + ".jpg"
                cv2.imwrite(path_save, img)
                # print("Image with kilobots detection save to : " + path_save)
            else:
                new_name = name_image + "_arena_" + str(p1) + "_" + str(p2)
                path_dir = path_data_image + name_video
                path_save = path_dir + "/" + new_name + ".jpg"
                cv2.imwrite(path_save, img)
                # print("Image with arena detection save to : " + path_save)

        else:
            return [], params, ""
    else:
        return [], params, ""

    return circles[0], params, path_save


def if_not_list(list_of_lists, sub_list):
    is_not_in_list_of_lists = True

    for lst in list_of_lists:
        if all(elem in lst for elem in sub_list):
            is_not_in_list_of_lists = False
            break

    if is_not_in_list_of_lists:
        return True

    return False


def count_elements(list_of_lists, dict_of_elements):
    for element in dict_of_elements.keys():
        dict_of_elements[element] = sum(x.count(element) for x in list_of_lists)

    return dict_of_elements


def point_in_circle(type_arena, point_x, point_y, center_inf_x, center_inf_y, radius_inf, center_sup_x, center_sup_y, radius_sup):
    if type_arena == "disk":
        # Calculate the distance between the point and the center of the circle
        distance = math.sqrt((point_x - center_inf_x) ** 2 + (point_y - center_inf_y) ** 2)

        # Check if the distance is less than or equal to the radius
        if distance <= radius_inf:
            return True
        else:
            return False

    elif type_arena == "ring":
        distance_inf = math.sqrt((point_x - center_inf_x) ** 2 + (point_y - center_inf_y) ** 2)
        distance_sup = math.sqrt((point_x - center_sup_x) ** 2 + (point_y - center_sup_y) ** 2)

        if distance_sup <= radius_sup and distance_inf >= radius_inf:
            return True
        else:
            return False


def get_position_arena(name_video, path_images, type_arena):
    x_circle_inf = -1
    y_circle_inf = -1
    radius_circle_inf = -1

    x_circle_sup = -1
    y_circle_sup = -1
    radius_circle_sup = -1

    flag_arena_detected = False
    flag_ring_inf = False
    flag_ring_sup = False

    liste_images_test = os.listdir(path_images)
    for img_test in liste_images_test:
        for i in range(30, 36):
            if type_arena == "disk":
                liste_circles_inf, parameters_inf, path_save_inf = detect_round_objects(name_video, img_test[:-4], 10, i, path_image=path_images, obj_type=type_arena)

                if parameters_inf is not None:
                    x_circle_inf = liste_circles_inf[0][0]
                    y_circle_inf = liste_circles_inf[0][1]
                    radius_circle_inf = liste_circles_inf[0][2]
                    flag_arena_detected = True
                    break

            elif type_arena == "ring":
                liste_circles_inf, parameters_inf, path_save_inf = detect_round_objects(name_video, img_test[:-4], 10, i, path_image=path_images, obj_type="disk")

                if parameters_inf is not None:
                    x_circle_inf = liste_circles_inf[0][0]
                    y_circle_inf = liste_circles_inf[0][1]
                    radius_circle_inf = liste_circles_inf[0][2]
                    flag_ring_inf = True

                liste_circles_sup, parameters_sup, path_save_sup = detect_round_objects(name_video, img_test[:-4], 20, i, path_image=path_images, obj_type=type_arena)

                if parameters_sup is not None:
                    x_circle_sup = liste_circles_sup[0][0]
                    y_circle_sup = liste_circles_sup[0][1]
                    radius_circle_sup = liste_circles_sup[0][2]
                    flag_ring_sup = True

                if flag_ring_inf and flag_ring_sup:
                    flag_arena_detected = True
                    break

        if flag_arena_detected:
            break

    return x_circle_inf, y_circle_inf, radius_circle_inf, x_circle_sup, y_circle_sup, radius_circle_sup


def analyse(name_video, nb_kilobots, type_arena, name_file=None, frames_per_second=1):
    f = open("./data/txt/"+name_file+"/"+name_video+".txt", "w")

    if name_file is None:
        path_frames = video_to_images(name_video, frames_per_second=frames_per_second)
    else:
        path_frames = video_to_images(name_video, name_file=name_file, frames_per_second=frames_per_second)

    # name of all the frames of the video
    name_frames = []
    liste_frames = os.listdir(path_frames)
    for frame in liste_frames:
        name_frames.append(frame[:-4])

    # detection of black edges for all frames od the video
    for frame in name_frames:
        image_be, path_img_be = detect_black_edges(name_video, frame, path_frames)

    # name of all the images of the black edges detection
    name_images = []
    liste_images = os.listdir(path_img_be)
    for img in liste_images:
        name_images.append(img[:-4])

    x_arena_inf, y_arena_inf, radius_arena_inf, x_arena_sup, y_arena_sup, radius_arena_sup = get_position_arena(name_video, path_img_be, type_arena)

    # enregistrer le nombre de kilobots et le centre de l'arène
    f.write(str(nb_kilobots)+","+str(x_arena_inf)+","+str(y_arena_inf)+","+str(radius_arena_inf)+","+str(x_arena_sup)+","+str(y_arena_sup)+","+str(radius_arena_sup)+"\n")

    all_params = []
    all_circles = []

    # kilobots detection
    for image in name_images:
        liste_params = []
        liste_circles = []

        flag_detection = False
        i = 10
        j = 10

        while not flag_detection:
            while not flag_detection:
                pos_circles, tuple_params, path_image_saved = detect_round_objects(name_video, image, i, j, nb_kilobots=nb_kilobots, path_image=path_img_be)

                # if we managed to detect the right number of circles
                if tuple_params is not None:
                    # check that all detected circles are in the arena
                    nb_correct = 0

                    for circle in pos_circles:
                        if point_in_circle(type_arena, circle[0], circle[1], x_arena_inf, y_arena_inf, radius_arena_inf, x_arena_sup, y_arena_sup, radius_arena_sup):
                            nb_correct += 1

                    if nb_correct == nb_kilobots:
                        flag_detection = True
                        liste_params.append(tuple_params)
                        liste_circles.append(pos_circles)

                        # save kilobot position
                        for circle in pos_circles:
                            line = f"{image[-7:-3]},{circle[0]},{circle[1]}\n"
                            f.write(line)
                    else:
                        os.remove(path_image_saved)
                        # print("False-positive, remove " + path_image_saved)

                if j == 20:  # timeout
                    break
                else:
                    j += 1

            if i == 50:  # timeout
                break
            else:
                i += 10

        # print(all_circles[0][:nb_k], liste_params)
        all_params.append(liste_params)
        all_circles.append(liste_circles)

    f.close()

    return all_circles, all_params


def analyse_all_video(name_file, path_videos, type_arenas, nb_kilobots):
    liste_videos = os.listdir(path_videos + name_file)

    for v in liste_videos:
        start = time.perf_counter()

        create_folder(name_file, v[:-4])

        for arena in type_arenas:
            if arena in v:
                analyse(v[:-4], nb_kilobots, arena, name_file=name_file, frames_per_second=2)
                break

        end = time.perf_counter()
        tps = end - start

        print(f"Temps d'exécution : {tps} s")


def create_folder(name_file, name_video):
    try:
        # creating a folder named data
        if not os.path.exists("data"):
            os.makedirs("data")
        if not os.path.exists("./data/images"):
            os.makedirs("./data/images")
        if not os.path.exists("./data/images/" + name_video):
            os.makedirs("./data/images/" + name_video)
        else:  # reset folder
            shutil.rmtree("./data/images/" + name_video)
            os.makedirs("./data/images/" + name_video)

        if not os.path.exists("./data/images/" + name_video + "/be"):
            os.makedirs("./data/images/" + name_video + "/be")
        if not os.path.exists("./data/images/" + name_video + "/rs"):
            os.makedirs("./data/images/" + name_video + "/rs")
        if not os.path.exists("./data/images/" + name_video + "/kb"):
            os.makedirs("./data/images/" + name_video + "/kb")

        if not os.path.exists("./data/txt"):
            os.makedirs("./data/txt")
        if not os.path.exists("./data/txt/" + name_file):
            os.makedirs("./data/txt/" + name_file)

        if not os.path.exists("./data/json"):
            os.makedirs("./data/json")
        if not os.path.exists("./data/json/" + name_file):
            os.makedirs("./data/json/" + name_file)

    except OSError:  # if not created then raise error
        print('Error: Creating directory of data')


def position_relative_to_center_arena(path_file):
    f = open(path_file, "r")
    lines = f.readlines()

    nb_kilobots = 0
    arena = ()
    liste_postions = []
    position_kilobots = {}

    for i, line in enumerate(lines):
        tmp = line.strip('\n').split(",")
        if i == 0:
            nb_kilobots = int(tmp[0])
            arena = (float(tmp[1]), float(tmp[2]), float(tmp[3]))
        elif i % nb_kilobots == 0:
            pos = (float(tmp[1]) - arena[0], float(tmp[2]) - arena[1])
            liste_postions.append(pos)
            position_kilobots[tmp[0]] = sorted(liste_postions)
            liste_postions = []
        else:
            pos = (float(tmp[1])-arena[0], float(tmp[2])-arena[1])
            liste_postions.append(pos)

    f.close()

    return position_kilobots


def convert_to_json(dict_positions, path_json):
    f = open(path_json+"_simulationStates.json", "w")

    dict_pos = {}
    dict_init = {}
    list_dict = []
    direction = 0

    for i, key in enumerate(dict_positions.keys()):
        dict_init["bot_states"] = []
        dict_init["ticks"] = int(key) * 16  # 32 ticks par sec (soit 16 pour une analyse de 2 frames par sec)
        for j, pos in enumerate(dict_positions[key]):
            dict_pos["ID"] = j
            dict_pos["direction"] = direction
            dict_pos["state"] = {}
            dict_pos["x_position"] = pos[0]
            dict_pos["y_position"] = pos[1]
            dict_init["bot_states"].append(dict_pos)
            dict_pos = {}
        list_dict.append(dict_init)
        dict_init = {}

    json.dump(list_dict, f)

    f.close()

    return list_dict


def end_state_json(list_dict, path_json):
    f = open(path_json+"_endState.json", "w")

    end_list = [list_dict[-1]]
    json.dump(end_list, f)

    f.close()

    return end_list


def convert_all(path_text, path_json):
    liste_txt = os.listdir(path_text)
    for txt_file in liste_txt:
        dict_p = position_relative_to_center_arena(path_text + txt_file)
        list_d = convert_to_json(dict_p, path_json + txt_file[:-4])
        end_state_json(list_d, path_json + txt_file[:-4])


if __name__ == '__main__':
    nb_k = 15
    all_arenas = ["disk", "ring"]
    videos_path = "./data/videos/"
    txt_path = "./data/txt/"
    json_path = "./data/json/"
    file = "2023-01-06"

    # analyse_all_video(file, videos_path, all_arenas, nb_k)
    # convert_all(txt_path+file+"/", json_path+file+"/")

    liste_files = os.listdir(videos_path)
    for file in liste_files:
        analyse_all_video(file, videos_path, all_arenas)
        convert_all(txt_path+file+"/", json_path+file+"/")




