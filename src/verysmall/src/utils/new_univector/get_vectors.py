def getVectors(w, h, step, get_vec, ball, obstacles):
    vectors = []

    for x in range(0, w, step):
        for y in range(0, h, step):
            vector = get_vec(x, y, ball, obstacles)
            vectors.append(vector)

    return vectors