#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <string>
#include <vector>
#include <cstdio>
#include <cstring>
#include <bit>
#include <sstream>
#include <exception>

#include <filesystem>
namespace fs = std::filesystem;

#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STBI_MSC_SECURE_CRT
#include <tiny_gltf.h>

using namespace std;

struct Weight {
    unsigned int joint;
    float weight;
};

struct Weights {
    vector<Weight> weights;
};

struct WeightsMap {
    map<unsigned int, float> weights;
};

class OssParser {
public:
    OssParser(const string &filename);
    ~OssParser();
    // Функции
    void toGLTF(const string& filename, const string& texture, unsigned int pose);
    void poseIt(unsigned int pose);
private:
    unsigned int num_frames;
    unsigned int frame_per_second;
    unsigned int num_joints;
    unsigned int num_vertices;
    unsigned int num_triangles;
    unsigned int num_uvs;
    // Arrays
    float* vertices;
    unsigned int* triangles;
    float* uvs;
    unsigned int* uvis;
    Weights* weights;
    float* joints;
};

template <typename T>
T max_array(T* a, int init, int offset, int num_a) {
    T result = a[init];
    init += offset;
    while (init < num_a) {
        if (result < a[init]) {
            result = a[init];
        }
        init += offset;
    }
    return result;
}

template <typename T>
T min_array(T* a, int init, int offset, int num_a) {
    T result = a[init];
    init += offset;
    while (init < num_a) {
        if (result > a[init]) {
            result = a[init];
        }
        init += offset;
    }
    return result;
}

template <typename T>
std::string toString(T val)
{
    std::ostringstream oss;
    oss << val;
    return oss.str();
}

void inverse_mat44(float* mat44, float* out_mat44) {
    float* a = mat44;
    out_mat44[0] = (a[14] * a[7] - a[15] * a[6]) * a[9] + (a[11] * a[6] - a[10] * a[7]) * a[13] + (a[10] * a[15] - a[11] * a[14]) * a[5];
    out_mat44[1] = (a[15] * a[2] - a[14] * a[3]) * a[9] + (a[10] * a[3] - a[11] * a[2]) * a[13] + (a[11] * a[14] - a[10] * a[15]) * a[1];
    out_mat44[2] = (a[13] * a[2] - a[1] * a[14]) * a[7] + (a[1] * a[15] - a[13] * a[3]) * a[6] + (a[14] * a[3] - a[15] * a[2]) * a[5];
    out_mat44[3] = (a[3] * a[6] - a[2] * a[7]) * a[9] + (a[10] * a[7] - a[11] * a[6]) * a[1] + (a[11] * a[2] - a[10] * a[3]) * a[5];
    float det = a[0] * out_mat44[0] + a[1] * out_mat44[1] + a[2] * out_mat44[2] + a[3] * out_mat44[3];
    out_mat44[0] /= det;
    out_mat44[1] /= det;
    out_mat44[2] /= det;
    out_mat44[3] /= det;
    out_mat44[4] = 1.f / det * (a[15] * a[6] - a[14] * a[7]) * a[8] + (a[10] * a[7] - a[11] * a[6]) * a[12] + (a[11] * a[14] - a[10] * a[15]) * a[4];
    out_mat44[5] = 1.f / det * (a[14] * a[3] - a[15] * a[2]) * a[8] + (a[11] * a[2] - a[10] * a[3]) * a[12] + (a[10] * a[15] - a[11] * a[14]) * a[0];
    out_mat44[6] = 1.f / det * (a[0] * a[14] - a[12] * a[2]) * a[7] + (a[12] * a[3] - a[0] * a[15]) * a[6] + (a[15] * a[2] - a[14] * a[3]) * a[4];
    out_mat44[7] = 1.f / det * (a[2] * a[7] - a[3] * a[6]) * a[8] + (a[11] * a[6] - a[10] * a[7]) * a[0] + (a[10] * a[3] - a[11] * a[2]) * a[4];
    out_mat44[8] = 1.f / det * (a[15] * a[4] - a[12] * a[7]) * a[9] + (a[13] * a[7] - a[15] * a[5]) * a[8] + (a[12] * a[5] - a[13] * a[4]) * a[11];
    out_mat44[9] = 1.f / det * (a[12] * a[3] - a[0] * a[15]) * a[9] + (a[1] * a[15] - a[13] * a[3]) * a[8] + (a[0] * a[13] - a[1] * a[12]) * a[11];
    out_mat44[10] = 1.f / det * (a[1] * a[12] - a[0] * a[13]) * a[7] + (a[0] * a[15] - a[12] * a[3]) * a[5] + (a[13] * a[3] - a[1] * a[15]) * a[4];
    out_mat44[11] = 1.f / det * (a[0] * a[7] - a[3] * a[4]) * a[9] + (a[3] * a[5] - a[1] * a[7]) * a[8] + (a[1] * a[4] - a[0] * a[5]) * a[11];
    out_mat44[12] = 1.f / det * (a[12] * a[6] - a[14] * a[4]) * a[9] + (a[14] * a[5] - a[13] * a[6]) * a[8] + (a[13] * a[4] - a[12] * a[5]) * a[10];
    out_mat44[13] = 1.f / det * (a[0] * a[14] - a[12] * a[2]) * a[9] + (a[13] * a[2] - a[1] * a[14]) * a[8] + (a[1] * a[12] - a[0] * a[13]) * a[10];
    out_mat44[14] = 1.f / det * (a[0] * a[13] - a[1] * a[12]) * a[6] + (a[12] * a[2] - a[0] * a[14]) * a[5] + (a[1] * a[14] - a[13] * a[2]) * a[4];
    out_mat44[15] = 1.f / det * (a[2] * a[4] - a[0] * a[6]) * a[9] + (a[1] * a[6] - a[2] * a[5]) * a[8] + (a[0] * a[5] - a[1] * a[4]) * a[10];
}

// Итак структура oss файла:
// int num_frames;     // число кадров анимации в файле
// int frame_rate;     // количество кадров в секунду
// int num_joints;     // количество костей
// int num_vertices;   // количество вершин
// int num_triangles;  // количество треугольников
// int num_uvs;        // количество uv координат на текстуре
// float vertices[num_vertices*3]; // вершины
// int triangles[num_triangles*3]; // индексы вершин треугольников
// float uvs[num_uvs*2];           // uv координаты текстуры
// int uvis[num_triangles*3];      // индексы текстурных координат для каждого треугольника
// Weights weights[num_vertices];  // веса костей при их движении
//     struct Weights {
//         int size;
//         Weight weights[size];
//     }
//     struct Weight {
//         int joint;    // номер кости, влияющей на данную вершину
//         float weight; // вес с которой она влияет на вершину
//     }
// float joints[7*num_frames*num_joints]; // информация о костях
//     7: float x, y, z; float qx, qy, qz, qw; // Вектор трансляции и кватернион поворота
OssParser::OssParser(const string &filename)
{
    FILE* f = fopen(filename.c_str(), "rb");
    fread(&num_frames, sizeof(int), 1, f);
    fread(&frame_per_second, sizeof(int), 1, f);
    fread(&num_joints, sizeof(int), 1, f);
    fread(&num_vertices, sizeof(int), 1, f);
    fread(&num_triangles, sizeof(int), 1, f);
    fread(&num_uvs, sizeof(int), 1, f);
    printf("Number of frames = %d\n", num_frames);
    printf("Frame per second = %d\n", frame_per_second);
    printf("Number of joints = %d\n", num_joints);
    printf("Number of vertices = %d\n", num_vertices);
    printf("Number of triangles = %d\n", num_triangles);
    printf("Number of uvs = %d\n", num_uvs);
    printf("    Position of Vertices = %x\n", ftell(f));
    vertices = new float[3 * num_vertices];
    fread(vertices, sizeof(float), 3 * num_vertices, f);
    triangles = new unsigned int[3 * num_triangles];
    printf("    Position of Triangles = %x\n", ftell(f));
    fread(triangles, sizeof(int), 3 * num_triangles, f);
    uvs = new float[2 * num_uvs];
    printf("    Position of UVs = %x\n", ftell(f));
    fread(uvs, sizeof(int), 2 * num_uvs, f);
    uvis = new unsigned int[3 * num_triangles];
    printf("    Position of UVIndexes = %x\n", ftell(f));
    fread(uvis, sizeof(int), 3 * num_triangles, f);
    Weight weight;
    unsigned int num_weights;
    weights = new Weights[num_vertices];
    printf("    Position of Weights = %x\n", ftell(f));
    for (unsigned int i = 0; i < num_vertices; ++i) {
        fread(&num_weights, sizeof(int), 1, f);
        //printf("      NumWeights for vertex %d = %d; JointsInfo = [", i, num_weights);
        Weights joint_weight;
        for (unsigned int j = 0; j < num_weights; ++j) {
            fread(&weight.joint, sizeof(int), 1, f);
            fread(&weight.weight, sizeof(float), 1, f);
            //printf("(%d, %f)", weight.joint, weight.weight);
            if (j != num_weights - 1) {
                //printf(", ");
            }
            joint_weight.weights.push_back(weight);
        }
        //printf("]\n");
        weights[i] = joint_weight;
    }
    printf("    Position of Joints = %x\n", ftell(f));
    joints = new float[7 * num_joints * num_frames];
    fread(joints, sizeof(float), 7 * num_joints * num_frames, f);
}

OssParser::~OssParser()
{
    delete[] vertices;
    delete[] triangles;
    delete[] uvs;
    delete[] uvis;
    delete[] joints;
    delete[] weights;
}

void OssParser::toGLTF(const string &filename, const string &texture, unsigned int pose)
{
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // У gltf оказалась проблема - они не могут хранить uv координаты отдельно от вертексов
    // Как следствие - 1 вертекс - 1 uv
    // В примере с сайта kronos - corset.gltf для разбиения текстуры на подобласти использовалось удвоение вертексов
    // границ Это требует:
    // 1. Найти уникальные вертексы с уникальными uv координатами, я решил это сделать в map
    ///////////////////////////////////////////////////////////////////////////////////////////////
    unsigned int max_triangles_in_vertex = 0;
    map<pair<unsigned int, unsigned int>, unsigned int> v_map;
    for (unsigned int i = 0; i < 3 * num_triangles; ++i) {
        unsigned int vA = triangles[i];  // индекс вертекса треугольника
        unsigned int uvA = uvis[i];      // uv индекс вертекса треугольника
        auto f_iter = v_map.find({vA, uvA});
        if (f_iter != v_map.end()) {  // если в массиве уже есть даный вертекс с данным uv индексом
            f_iter->second +=
                1;  // считаем сколько таких вершин (в одной вершине могут сходиться несколько треугольников
            if (f_iter->second > max_triangles_in_vertex) {
                max_triangles_in_vertex = f_iter->second;
            }
        } else {
            v_map.insert({{vA, uvA}, 1});  // иначе добавляем такую пару в массив
        }
    }
    printf("max f_iter->second = %d\n", max_triangles_in_vertex);
    unsigned int n_vertices = (unsigned int)v_map.size();  // Истинный размер нового массива вертексов!
    float* f_vertices = new float[n_vertices * 3];
    float* f_uvs = new float[n_vertices * 2];
    unsigned int* f_triangles = new unsigned int[num_triangles * 3];
    Weights* f_weights = new Weights[n_vertices];
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // 2. Заполнить новый массив вертексами из map и записать индекс в качестве значения map, чтобы возвращать по
    // уникальной паре индекс в новом массиве
    //    Также здесь можно заполнить буфер текстурных координат
    ///////////////////////////////////////////////////////////////////////////////////////////////
    unsigned int ind = 0;
    for (auto i = v_map.begin(); i != v_map.end(); i++) {
        f_vertices[3 * ind] = vertices[3 * i->first.first];
        f_vertices[3 * ind + 1] = vertices[3 * i->first.first + 1];
        f_vertices[3 * ind + 2] = vertices[3 * i->first.first + 2];
        f_uvs[2 * ind] = uvs[2 * i->first.second];
        f_uvs[2 * ind + 1] = 1 - uvs[2 * i->first.second + 1];
        map<int, float> s_map;
        for (auto j = weights[i->first.first].weights.begin(); j < weights[i->first.first].weights.end(); ++j) {
            auto f_iter = s_map.find(j->joint);
            if (f_iter != s_map.end()) {  // если в массиве уже есть даный вертекс с данным uv индексом
                f_iter->second +=
                    j->weight;  // считаем сколько таких вершин (в одной вершине могут сходиться несколько треугольников
            } else {
                s_map.insert({j->joint, j->weight});  // иначе добавляем такую пару в массив
            }
        }
        Weights t_weights;
        for (auto j = s_map.begin(); j != s_map.end(); ++j) {
            Weight t_weight;
            t_weight.joint = j->first;
            t_weight.weight = j->second;
            t_weights.weights.push_back(t_weight);
        }
        f_weights[ind] = t_weights;
        i->second = ind;  // заменяем в v_map количество треугольников, которые сходятся в данном вертексе на индекс в
                          // v_map, чтобы потом не бегать по v_map, когда будем переопределять треугольники
        ++ind;
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // 3. Заполнить буфер индексов треугольников
    ///////////////////////////////////////////////////////////////////////////////////////////////
    for (unsigned int i = 0; i < 3 * num_triangles; ++i) {
        unsigned int vA = triangles[i];
        unsigned int uvA = uvis[i];
        // printf("%d\n", v_map[{vA, uvA}]);
        f_triangles[i] = v_map[{vA, uvA}];
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // 4. Вспомогательные расчёты для учёта как выглядит скелет
    ///////////////////////////////////////////////////////////////////////////////////////////////
    WeightsMap* weights_map = new WeightsMap[num_joints]; // Массив с соединениями костей
    float* center_masses = new float[num_joints * 3];     // Центры масс костей
    float* masses = new float[num_joints];                // Массы для расчёта центра масс костей (равны сумме весов)
    map<pair<unsigned int, unsigned int>, unsigned int> connections;
    for (unsigned int i = 0; i < num_joints; ++i) {
        weights_map[i] = WeightsMap();
        masses[i] = 0;
        center_masses[3 * i] = 0;
        center_masses[3 * i + 1] = 0;
        center_masses[3 * i + 2] = 0;
    }
    unsigned int max_weights = 0;
    for (unsigned int i = 0; i < num_vertices; ++i) {
        if (max_weights < weights[i].weights.size()) {
            max_weights = (unsigned int)weights[i].weights.size();
        }
        for (unsigned int j = 0; j < weights[i].weights.size(); ++j) {
            for (unsigned int k = j + 1; k < weights[i].weights.size(); ++k) {
                unsigned int joint1 = weights[i].weights[j].joint;
                unsigned int joint2 = weights[i].weights[k].joint;
                if (joint2 != joint1) {
                    if (joint2 < joint1) {
                        swap(joint1, joint2);
                    }
                    auto iter2 = connections.find({joint1, joint2});
                    if (iter2 != connections.end()) {
                        iter2->second += 1;
                    } else {
                        connections.insert({{joint1, joint2}, 1});
                    }
                }
            }
            unsigned int joint = weights[i].weights[j].joint;
            float weight = weights[i].weights[j].weight;
            center_masses[joint * 3] += weight * vertices[3 * i];
            center_masses[joint * 3 + 1] += weight * vertices[3 * i + 1];
            center_masses[joint * 3 + 2] += weight * vertices[3 * i + 2];
            masses[joint] += weight;
            auto iter = weights_map[joint].weights.find(i);
            if (iter != weights_map[joint].weights.end()) {
                iter->second += weight;
            } else {
                weights_map[joint].weights.insert({i, weight});
            }
        }
    }
    for (unsigned int i = 0; i < num_joints; ++i) {
        if (masses[i] > 1e-6) {
            // center_masses[3 * i]     /= masses[i];
            // center_masses[3 * i + 1] /= masses[i];
            // center_masses[3 * i + 2] /= masses[i];
            center_masses[3 * i] = joints[pose * num_joints * 7 + i * 7];
            center_masses[3 * i + 1] = joints[pose * num_joints * 7 + i * 7 + 1];
            center_masses[3 * i + 2] = joints[pose * num_joints * 7 + i * 7 + 2];
        }
    }
    //for (auto iter = connections.begin(); iter != connections.end(); ++iter) {
    //    printf("Connection (%d %d) %d\n", iter->first.first, iter->first.second, iter->second);
    //}
    for (unsigned int i = 0; i < num_joints; ++i) {
        // printf("Num vertices in joints[%d]: %lld\n", i, weights_map[i].weights.size());
        // printf("Joint center mass = (%e, %e %e)\n", center_masses[3 * i], center_masses[3 * i + 1], center_masses[3 *
        // i + 2]); printf("Joint position = (%e %e %e), rotation = (%e %e %e %e)\n",
        //     joints[7 * i], joints[7 * i + 1], joints[7 * i + 2],
        //     joints[7 * i + 3], joints[7 * i + 4], joints[7 * i + 5], joints[7 * i + 6]
        //);
        float* joint_pos = joints + num_joints * 7 * 505;
        // printf("Joint position = (%e %e %e), rotation = (%e %e %e %e)\n",
        //     joint_pos[7 * i], joint_pos[7 * i + 1], joint_pos[7 * i + 2],
        //     joint_pos[7 * i + 3], joint_pos[7 * i + 4], joint_pos[7 * i + 5], joint_pos[7 * i + 6]
        //);
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // 5. Веса и индексы костей вертексов
    // В gltf они хранятся для каждого вертекса по 4 штуки, поэтому сначала нужно вычислить их
    // для каждого вертекса и каждой кости, определить сколько массивов по 4 штуки получится и
    // только потом всё записать в accessor'ы
    ///////////////////////////////////////////////////////////////////////////////////////////////
    unsigned int max_weights2 = 0;
    for (unsigned int i = 0; i < n_vertices; ++i) {
        if (max_weights2 < (unsigned int)f_weights[i].weights.size()) {
            max_weights2 = (unsigned int)f_weights[i].weights.size();
        }
    }
    unsigned int num_jw_arrays = (max_weights - 1) / 4 + 1;
    unsigned char** s_joints = new unsigned char*[num_jw_arrays];
    float** s_weights = new float*[num_jw_arrays];
    for (unsigned int i = 0; i < num_jw_arrays; ++i) {
        s_joints[i] = new unsigned char[4 * n_vertices];
        s_weights[i] = new float[4 * n_vertices];
        for (unsigned int j = 0; j < n_vertices; ++j) {
            s_joints[i][4 * j] = 0;
            s_joints[i][4 * j + 1] = 0;
            s_joints[i][4 * j + 2] = 0;
            s_joints[i][4 * j + 3] = 0;
            s_weights[i][4 * j] = 0.0f;
            s_weights[i][4 * j + 1] = 0.0f;
            s_weights[i][4 * j + 2] = 0.0f;
            s_weights[i][4 * j + 3] = 0.0f;
        }
    }
    for (unsigned int i = 0; i < n_vertices; ++i) {
        for (unsigned int j = 0; j < f_weights[i].weights.size(); ++j) {
            s_joints[j / 4][4 * i + j % 4] = (unsigned char)(f_weights[i].weights[j].joint % 256);
            s_weights[j / 4][4 * i + j % 4] = f_weights[i].weights[j].weight;
        }
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Блок вывода информации о подготовленных данных
    ////////////////////////////////////////////////////////////////////////////////////////////////
    printf("max_weights = %d\n", max_weights);
    printf("n_vertices size = %d\n", n_vertices);
    printf("f_vertices size = %d\n", n_vertices * 3);
    printf("f_triangles size = %d\n", num_triangles * 3);
    printf("f_uvs size = %d\n", n_vertices * 2);
    printf("Size of map = %lld sizeof(float) = %lld\n", v_map.size(), sizeof(float));
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Подготовка данных завершена, можно заполнять данные gltf
    ////////////////////////////////////////////////////////////////////////////////////////////////
    tinygltf::Model m;       // Файл gltf
    tinygltf::Scene scene;   // Сцена в файле
    tinygltf::Mesh mesh;     // Меш в сцене
    // В каждом gltf mesh есть primitive (определяет как мы рисуем треугольники) он может быть и не один
    tinygltf::Primitive primitive;  
    primitive.indices = 1;
    primitive.attributes["POSITION"] = 0;
    primitive.attributes["TEXCOORD_0"] = 2;
    string str_joints = "JOINTS_";
    string str_weights = "WEIGHTS_";
    for (unsigned i = 0; i < num_jw_arrays; ++i) {
      primitive.attributes[(str_joints + toString(i)).c_str()] = 3 + 2 * i;
      primitive.attributes[(str_weights + toString(i)).c_str()] = 3 + 2 * i + 1;
    }
    primitive.material = 0;
    primitive.mode = TINYGLTF_MODE_TRIANGLES;

    mesh.primitives.push_back(primitive);

    // Непонятная штука, видимо задает фичи gltf
    tinygltf::Asset asset;
    asset.version = "2.0";
    asset.generator = "tinygltf";
    m.asset = asset;

    m.meshes.push_back(mesh);
    // m.meshes.push_back(cmesh);
    /////////////////////////////////////////////////////////////////////////////////////////////////
    // Буферы
    /////////////////////////////////////////////////////////////////////////////////////////////////
    // Вертексные буферы
    tinygltf::Buffer b_vertices;
    tinygltf::Buffer b_triangles;
    tinygltf::Buffer b_uvs;
    tinygltf::BufferView bv_vertices;
    tinygltf::BufferView bv_triangles;
    tinygltf::BufferView bv_uvs;
    tinygltf::Accessor a_vertices;
    tinygltf::Accessor a_triangles;
    tinygltf::Accessor a_uvs;

    b_vertices.data.resize(n_vertices * 3 * sizeof(float));
    memcpy(&b_vertices.data[0], &f_vertices[0], n_vertices * 3 * sizeof(float));
    b_triangles.data.resize(num_triangles * 3 * sizeof(unsigned int));
    memcpy(&b_triangles.data[0], &f_triangles[0], num_triangles * 3 * sizeof(unsigned int));
    b_uvs.data.resize(n_vertices * 2 * sizeof(float));
    memcpy(&b_uvs.data[0], &f_uvs[0], n_vertices * 2 * sizeof(float));

    m.buffers.push_back(b_vertices);
    m.buffers.push_back(b_triangles);
    m.buffers.push_back(b_uvs);

    bv_vertices.buffer = 0;
    bv_vertices.byteOffset = 0;
    bv_vertices.byteLength = n_vertices * 3 * sizeof(float);
    bv_vertices.target = TINYGLTF_TARGET_ARRAY_BUFFER;

    a_vertices.bufferView = 0;
    a_vertices.byteOffset = 0;
    a_vertices.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
    a_vertices.count = n_vertices;
    a_vertices.type = TINYGLTF_TYPE_VEC3;
    float xmin = min_array<float>(vertices, 0, 3, num_vertices * 3);
    float ymin = min_array<float>(vertices, 1, 3, num_vertices * 3);
    float zmin = min_array<float>(vertices, 2, 3, num_vertices * 3);
    float xmax = max_array<float>(vertices, 0, 3, num_vertices * 3);
    float ymax = max_array<float>(vertices, 1, 3, num_vertices * 3);
    float zmax = max_array<float>(vertices, 2, 3, num_vertices * 3);
    a_vertices.maxValues = {xmax, ymax, zmax};
    a_vertices.minValues = {xmin, ymin, zmin};

    bv_triangles.buffer = 1;
    bv_triangles.byteOffset = 0;
    bv_triangles.byteLength = num_triangles * 3 * sizeof(unsigned int);
    bv_triangles.target = TINYGLTF_TARGET_ELEMENT_ARRAY_BUFFER;

    a_triangles.bufferView = 1;
    a_triangles.byteOffset = 0;
    a_triangles.componentType = TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT;
    a_triangles.count = num_triangles * 3;
    a_triangles.type = TINYGLTF_TYPE_SCALAR;
    a_triangles.maxValues.push_back(n_vertices - 1);
    a_triangles.minValues.push_back(0);

    bv_uvs.buffer = 2;
    bv_uvs.byteOffset = 0;
    bv_uvs.byteLength = n_vertices * 2 * sizeof(float);
    bv_uvs.target = TINYGLTF_TARGET_ARRAY_BUFFER;

    a_uvs.bufferView = 2;
    a_uvs.byteOffset = 0;
    a_uvs.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
    a_uvs.count = n_vertices;
    a_uvs.type = TINYGLTF_TYPE_VEC2;
    float uv_xmin = min_array<float>(f_uvs, 0, 2, n_vertices * 2);
    float uv_ymin = min_array<float>(f_uvs, 1, 2, n_vertices * 2);
    float uv_xmax = max_array<float>(f_uvs, 0, 2, n_vertices * 2);
    float uv_ymax = max_array<float>(f_uvs, 1, 2, n_vertices * 2);
    a_uvs.maxValues = {uv_xmax, uv_ymax};
    a_uvs.minValues = {uv_xmin, uv_ymin};

    m.bufferViews.push_back(bv_vertices);
    m.bufferViews.push_back(bv_triangles);
    m.bufferViews.push_back(bv_uvs);

    m.accessors.push_back(a_vertices);   // 0
    m.accessors.push_back(a_triangles);  // 1
    m.accessors.push_back(a_uvs);        // 2

    // Joint & Weight буферы
    std::vector<tinygltf::Buffer> b_joint_weights;
    std::vector<tinygltf::BufferView> bv_weights;
    std::vector<tinygltf::BufferView> bv_joints;
    std::vector<tinygltf::Accessor> a_weights;
    std::vector<tinygltf::Accessor> a_joints;

    for (unsigned i = 0; i < num_jw_arrays; ++i) {
        // Заполняем буферы
        tinygltf::Buffer b_joint_weight;
        b_joint_weight.data.resize(n_vertices * 4 * sizeof(unsigned char) + n_vertices * 4 * sizeof(float));
        memcpy(&b_joint_weight.data[0], &s_joints[i][0], n_vertices * 4 * sizeof(unsigned char));
        memcpy(&b_joint_weight.data[n_vertices * 4 * sizeof(unsigned char)], &s_weights[i][0],
               n_vertices * 4 * sizeof(float));

        b_joint_weights.push_back(b_joint_weight);

        // Говорим программе, где искать данные
        tinygltf::BufferView bv_joint;
        tinygltf::BufferView bv_weight;
        bv_joint.buffer = i + 3;
        bv_joint.byteOffset = 0;
        bv_joint.byteLength = n_vertices * 4 * sizeof(unsigned char);
        bv_joint.target = TINYGLTF_TARGET_ARRAY_BUFFER;

        bv_weight.buffer = i + 3;
        bv_weight.byteOffset = n_vertices * 4 * sizeof(unsigned char);
        bv_weight.byteLength = n_vertices * 4 * sizeof(float);
        bv_weight.target = TINYGLTF_TARGET_ARRAY_BUFFER;

        bv_joints.push_back(bv_joint);
        bv_weights.push_back(bv_weight);

        // Объясняем, как их просматривать
        tinygltf::Accessor a_joint;
        tinygltf::Accessor a_weight;

        a_joint.bufferView = 3 + 2 * i;
        a_joint.byteOffset = 0;
        a_joint.componentType = TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE;
        a_joint.count = n_vertices;
        a_joint.type = TINYGLTF_TYPE_VEC4;
        double xmin = (double)min_array<unsigned char>(s_joints[i], 0, 4, n_vertices * 4);
        double ymin = (double)min_array<unsigned char>(s_joints[i], 1, 4, n_vertices * 4);
        double zmin = (double)min_array<unsigned char>(s_joints[i], 2, 4, n_vertices * 4);
        double tmin = (double)min_array<unsigned char>(s_joints[i], 3, 4, n_vertices * 4);
        double xmax = (double)max_array<unsigned char>(s_joints[i], 0, 4, n_vertices * 4);
        double ymax = (double)max_array<unsigned char>(s_joints[i], 1, 4, n_vertices * 4);
        double zmax = (double)max_array<unsigned char>(s_joints[i], 2, 4, n_vertices * 4);
        double tmax = (double)max_array<unsigned char>(s_joints[i], 3, 4, n_vertices * 4);
        a_joint.maxValues = {xmax, ymax, zmax, tmax};
        a_joint.minValues = {xmin, ymin, zmin, tmin};

        a_weight.bufferView = 3 + 2 * i + 1;  // 3 + 2 * (n_jw - 1) + 1 = 2 * n_jw +2
        a_weight.byteOffset = 0;
        a_weight.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
        a_weight.count = n_vertices;
        a_weight.type = TINYGLTF_TYPE_VEC4;
        xmin = (double)min_array<float>(s_weights[i], 0, 4, n_vertices * 4);
        ymin = (double)min_array<float>(s_weights[i], 1, 4, n_vertices * 4);
        zmin = (double)min_array<float>(s_weights[i], 2, 4, n_vertices * 4);
        tmin = (double)min_array<float>(s_weights[i], 3, 4, n_vertices * 4);
        xmax = (double)max_array<float>(s_weights[i], 0, 4, n_vertices * 4);
        ymax = (double)max_array<float>(s_weights[i], 1, 4, n_vertices * 4);
        zmax = (double)max_array<float>(s_weights[i], 2, 4, n_vertices * 4);
        tmax = (double)max_array<float>(s_weights[i], 3, 4, n_vertices * 4);
        a_weight.maxValues = {xmax, ymax, zmax, tmax};
        a_weight.minValues = {xmin, ymin, zmin, tmin};

        a_joints.push_back(a_joint);
        a_weights.push_back(a_weight);
    }

    for (unsigned i = 0; i < num_jw_arrays; ++i) {
        m.buffers.push_back(b_joint_weights[i]);
        m.bufferViews.push_back(bv_joints[i]);
        m.bufferViews.push_back(bv_weights[i]);
        m.accessors.push_back(a_joints[i]);   // 2 + 2*i
        m.accessors.push_back(a_weights[i]);  // 3 + 2*i; max = 3 + 2 * (num_jw_arrays - 1) = 1 + 2 * num_jw_arrays
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Буферы анимации
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Анимация в oss не содержит отдельного массива времен, вместо этого задано количество кадров
    // в секунду frames_per_second и количество кадров в файле, gltf наоборот требует явного 
    // задания времен для анимации
    // С анимацией пока не ясно, как идёт укладка кадров в oss
    // я сначала полагал, что в chimera.oss лежит 511 кадров по 89 костей по 1 трансляции и 
    // кватерниону последовательно, но похоже я ошибся и лежит 89 костей по 511 кадров по 1 
    // трансляции и кватерниону последовательно
    // GLTF, как оказалось, не умеет работать со всеми костями одновременно
    // Вместо этого анимация каждой кости требует отдельного accessor'а  и bufferView
    ///////////////////////////////////////////////////////////////////////////////////////////////

    // Времена кадров
    float* times = new float[num_frames];
    for (unsigned int i = 0; i < num_frames; ++i) {
        times[i] = 1.0f * i / frame_per_second;
    }

    tinygltf::Buffer b_times;
    tinygltf::BufferView bv_times;
    tinygltf::Accessor a_times;

    b_times.data.resize(num_frames * sizeof(float));
    memcpy(&b_times.data[0], &times[0], num_frames * sizeof(float));

    bv_times.buffer = num_jw_arrays + 3;
    bv_times.byteOffset = 0;
    bv_times.byteLength = num_frames * sizeof(float);

    a_times.bufferView = 2 * num_jw_arrays + 3;
    a_times.byteOffset = 0;
    a_times.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
    a_times.count = num_frames;
    a_times.type = TINYGLTF_TYPE_SCALAR;
    float tmin = min_array<float>(times, 0, 1, num_frames);
    float tmax = max_array<float>(times, 0, 1, num_frames);
    a_times.maxValues = {tmax};
    a_times.minValues = {tmin};

    m.buffers.push_back(b_times);
    m.bufferViews.push_back(bv_times);
    m.accessors.push_back(a_times);

    // Буферы кватернионов и трансляций
    tinygltf::Buffer b_anims;
    std::vector<tinygltf::BufferView> bv_anims_rotations;
    std::vector<tinygltf::BufferView> bv_anims_translations;
    std::vector<tinygltf::Accessor> a_anims_rotations;
    std::vector<tinygltf::Accessor> a_anims_translations;

    // buffer будет общий (Общий нельзя! Ошибка - bufferView.byteStride must not be defined for buffer views used by animation sampler accessors.)
    float* translations = new float[3 * num_joints * num_frames];
    float* rotations = new float[4 * num_joints * num_frames];
    for (unsigned i = 0; i<num_joints*num_frames; ++i) {
        translations[3 * i] = joints[7 * i];
        translations[3 * i + 1] = joints[7 * i + 1];
        translations[3 * i + 2] = joints[7 * i + 2];
        rotations[4 * i] = -joints[7 * i + 3];
        rotations[4 * i + 1] = -joints[7 * i + 4];
        rotations[4 * i + 2] = -joints[7 * i + 5];
        rotations[4 * i + 3] = joints[7 * i + 6];
    }
    b_anims.data.resize(7 * num_joints * num_frames * sizeof(float));
    memcpy(&b_anims.data[0], &translations[0], 3 * num_joints * num_frames * sizeof(float));
    memcpy(&b_anims.data[3 * num_joints * num_frames * sizeof(float)], &rotations[0],
           4 * num_joints * num_frames * sizeof(float));
    m.buffers.push_back(b_anims);

    delete [] translations;
    delete [] rotations;

    // Проходимся по костям и создаём для каждой кости bufferView и accessor отдельно для 
    // трансляций, отдельно для кватернионов
    tinygltf::Animation anim;
    unsigned index_buffer = (unsigned) m.buffers.size() - 1;
    unsigned index_times = (unsigned) m.accessors.size() - 1;
    for (unsigned i = 0; i < num_joints; ++i) {
        unsigned index_buffer_view = (unsigned) m.bufferViews.size();
        unsigned index_accessor = (unsigned) m.accessors.size();
        unsigned index_sampler = (unsigned) anim.samplers.size();

        tinygltf::BufferView bv_anims_rotation;
        tinygltf::BufferView bv_anims_translation;

        bv_anims_rotation.buffer = index_buffer;
        bv_anims_rotation.byteOffset = 3 * i * num_frames * sizeof(float);
        bv_anims_rotation.byteLength = 3 * num_frames * sizeof(float);

        bv_anims_translation.buffer = index_buffer;
        bv_anims_translation.byteOffset = 3 * num_joints * num_frames * sizeof(float) + 4 * i * num_frames * sizeof(float);
        bv_anims_translation.byteLength = 4 * num_frames * sizeof(float);

        tinygltf::Accessor a_anims_translation;
        tinygltf::Accessor a_anims_rotation;

        a_anims_translation.bufferView = index_buffer_view;
        a_anims_translation.byteOffset = 0;
        a_anims_translation.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
        a_anims_translation.count = num_frames;
        a_anims_translation.type = TINYGLTF_TYPE_VEC3;

        a_anims_rotation.bufferView = index_buffer_view + 1;
        a_anims_rotation.byteOffset = 0;
        a_anims_rotation.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
        a_anims_rotation.count = num_frames;
        a_anims_rotation.type = TINYGLTF_TYPE_VEC4;

        m.bufferViews.push_back(bv_anims_rotation);
        m.bufferViews.push_back(bv_anims_translation);
        m.accessors.push_back(a_anims_translation);
        m.accessors.push_back(a_anims_rotation);

        // Готовим анимации - samler - безье кривая, от времени
        tinygltf::AnimationSampler anim_smp1;
        anim_smp1.interpolation = "LINEAR";
        anim_smp1.input = index_times;   // accessor to times
        anim_smp1.output = index_accessor;  // accessor to translations

        tinygltf::AnimationSampler anim_smp2;
        anim_smp2.interpolation = "LINEAR";
        anim_smp2.input = index_times;   // accessor to times
        anim_smp2.output = index_accessor + 1;  // accessor to rotations

        anim.samplers.push_back(anim_smp1);
        anim.samplers.push_back(anim_smp2);

        tinygltf::AnimationChannel anim_chan1;
        anim_chan1.sampler = index_sampler;
        anim_chan1.target_node = i;
        anim_chan1.target_path = "translation";

        tinygltf::AnimationChannel anim_chan2;
        anim_chan2.sampler = index_sampler + 1;
        anim_chan2.target_node = i;
        anim_chan2.target_path = "rotation";

        anim.channels.push_back(anim_chan1);
        anim.channels.push_back(anim_chan2);
    }

    m.animations.push_back(anim);

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Отдельная проблема это InverseBindMatrices
    // К сожалению, но код ниже работает неправильно - иногда det матрицы равен 0. Обращение даёт 
    // бред, к счастью, как оказалось она и не нужна!
    ///////////////////////////////////////////////////////////////////////////////////////////////

    // tinygltf::Buffer ibm_buffer;
    // tinygltf::BufferView ibm_buffer_view;
    // tinygltf::Accessor ibm_accessor;
    // 
    // float* ibm = new float[16 * num_joints];
    // float inv_ibm[16];
    // //float identity[7] = {0, 0, 0, 0, 0, 0, 1};
    // for (unsigned int i = 0; i<num_joints; ++i) {
    //     float* quat = &joints[num_frames * 7 * i + 3];
    //     float* translate = &joints[num_frames * 7 * i];
    //     float s = 1.0f / (quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3]);
    //     unsigned int x = 0;
    //     unsigned int y = 1;
    //     unsigned int z = 2;
    //     unsigned int u = 3;
    //     inv_ibm[0]  = 1 - 2 * s * (quat[y] * quat[y] + quat[z] * quat[z]);
    //     inv_ibm[1]  = 2 * s * (quat[x] * quat[y] - quat[u] * quat[z]);
    //     inv_ibm[2]  = 2 * s * (quat[x] * quat[z] + quat[u] * quat[y]);
    //     inv_ibm[3]  = 0.0f; // translate[x];
    //     inv_ibm[4]  = 2 * s * (quat[x] * quat[y] + quat[u] * quat[z]);
    //     inv_ibm[5]  = 1 - 2 * s * (quat[x] * quat[x] + quat[z] * quat[z]);
    //     inv_ibm[6]  = 2 * s * (quat[y] * quat[z] - quat[u] * quat[x]);
    //     inv_ibm[7]  = 0.0f; // translate[y];
    //     inv_ibm[8]  = 2 * s * (quat[x] * quat[z] - quat[u] * quat[y]);
    //     inv_ibm[9]  = 2 * s * (quat[y] * quat[z] + quat[u] * quat[x]);
    //     inv_ibm[10] = 1 - 2 * s * (quat[x] * quat[x] + quat[y] * quat[y]);
    //     inv_ibm[11] = 0.0f; // translate[z];
    //     inv_ibm[12] = translate[x];
    //     inv_ibm[13] = translate[y];
    //     inv_ibm[14] = translate[z];
    //     inv_ibm[15] = 1.0f;
    //     inverse_mat44(inv_ibm, &ibm[16 * i]);
    // }
    // ibm_buffer.data.resize(16 * num_joints * sizeof(float));
    // memcpy(&ibm_buffer.data[0], &ibm[0], 16 * num_joints * sizeof(float));
    // m.buffers.push_back(ibm_buffer);
    // 
    // ibm_buffer_view.buffer = (unsigned) m.buffers.size() - 1;
    // ibm_buffer_view.byteOffset = 0;
    // ibm_buffer_view.byteLength = 16 * num_joints * sizeof(float);
    // m.bufferViews.push_back(ibm_buffer_view);
    // 
    // ibm_accessor.bufferView = (unsigned) m.bufferViews.size() - 1;
    // ibm_accessor.byteOffset = 0;
    // ibm_accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
    // ibm_accessor.count = num_joints;
    // ibm_accessor.type = TINYGLTF_TYPE_MAT4;
    // m.accessors.push_back(ibm_accessor);
    // 
    // unsigned ibm_accessor_index = (unsigned) m.accessors.size() - 1;

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Скелет
    ///////////////////////////////////////////////////////////////////////////////////////////////
    
    tinygltf::Node main_node;  // Структура скелета передаётся в node (пока не используется)
    main_node.mesh = 0;
    main_node.skin = 0;
    char strbuf[1024];

    for (unsigned int i = 0; i < num_joints; ++i) {
        tinygltf::Node node;  // Структура скелета передаётся в node (пока не используется)
        node.rotation =   
            //{0.0f, 0.0f, 0.0f, 1.0f};
            {
                joints[num_frames * 7 * i + 3], 
                joints[num_frames * 7 * i + 4], 
                joints[num_frames * 7 * i + 5], 
                joints[num_frames * 7 * i + 6] 
            };
        node.translation =  //{ center_masses[3 * i], center_masses[3 * i + 1], center_masses[3 * i + 2] }; 
            //{0.0f, 0.0f, 0.0f};
            {
                joints[num_frames * 7 * i + 0], 
                joints[num_frames * 7 * i + 1], 
                joints[num_frames * 7 * i + 2]
            };
        sprintf(strbuf, "Joint.%03d", i);
        node.name = std::string(strbuf);
        main_node.children.push_back(i);
        m.nodes.push_back(node);
    }

    m.nodes.push_back(main_node);
    scene.nodes.push_back((unsigned)m.nodes.size() - 1);
    m.scenes.push_back(scene);

    tinygltf::Skin skin;
    //skin.inverseBindMatrices = ibm_accessor_index; // Не нужна!
    for (unsigned int i = 0; i < num_joints; ++i) {
        skin.joints.push_back(i);
    }
    m.skins.push_back(skin);

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Материалы
    ///////////////////////////////////////////////////////////////////////////////////////////////

    tinygltf::Material mat;
    mat.pbrMetallicRoughness.baseColorTexture.index = 0;
    mat.doubleSided = true;
    m.materials.push_back(mat);

    tinygltf::Texture tex;
    tex.sampler = 0;
    tex.source = 0;
    m.textures.push_back(tex);

    tinygltf::Image img;
    img.uri = texture;
    m.images.push_back(img);

    tinygltf::Sampler smp;
    smp.magFilter = TINYGLTF_TEXTURE_FILTER_LINEAR;
    smp.minFilter = TINYGLTF_TEXTURE_FILTER_LINEAR_MIPMAP_LINEAR;
    smp.wrapS = TINYGLTF_TEXTURE_WRAP_REPEAT;
    smp.wrapT = TINYGLTF_TEXTURE_WRAP_REPEAT;
    m.samplers.push_back(smp);

    //m.skins.push_back(skin);

    tinygltf::TinyGLTF gltf;

    gltf.WriteGltfSceneToFile(&m, filename.c_str(),
        true, // embedImages
        true, // embedBuffers
        true, // pretty print
        false); // write binary

    delete[] f_triangles;
    delete[] f_vertices;
    delete[] f_uvs;
    delete[] weights_map;
    delete[] center_masses;      // Центры масс костей
    delete[] masses;
    for (unsigned int i = 0; i < num_jw_arrays; ++i) {
        delete[] s_joints[i];
        delete[] s_weights[i];
    }
    delete[] s_joints;
    delete[] s_weights;
    delete[] times;
    //delete[] ibm;
}

void apply_tranform(float* output3, float* vertex3, float* transform7, float* joint_pos, float weight) 
{
    float* quat = transform7 + 3;
    float* translate = transform7;
    float s = 1.0f/(quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3]);
    //printf("Quaternion (%e, %e, %e, %e) norm = %e\n", quat[0], quat[1], quat[2], quat[3], s);
    float matrix[9];
    unsigned int x = 0;
    unsigned int y = 1;
    unsigned int z = 2;
    unsigned int u = 3;
    matrix[0] = 1 - 2 * s * (quat[y] * quat[y] + quat[z] * quat[z]);
    matrix[1] = 2 * s * (quat[x] * quat[y] - quat[u] * quat[z]);
    matrix[2] = 2 * s * (quat[x] * quat[z] + quat[u] * quat[y]);
    matrix[3] = 2 * s * (quat[x] * quat[y] + quat[u] * quat[z]);
    matrix[4] = 1 - 2 * s * (quat[x] * quat[x] + quat[z] * quat[z]);
    matrix[5] = 2 * s * (quat[y] * quat[z] - quat[u] * quat[x]);
    matrix[6] = 2 * s * (quat[x] * quat[z] - quat[u] * quat[y]);
    matrix[7] = 2 * s * (quat[y] * quat[z] + quat[u] * quat[x]);
    matrix[8] = 1 - 2 * s * (quat[x] * quat[x] + quat[y] * quat[y]);
    float v3[3];
    v3[0] = vertex3[0] + translate[0];
    v3[1] = vertex3[1] + translate[1];
    v3[2] = vertex3[2] + translate[2];
    output3[0] += weight * (matrix[0] * v3[0] + matrix[1] * v3[1] + matrix[2] * v3[2] - translate[0]);
    output3[1] += weight * (matrix[3] * v3[0] + matrix[4] * v3[1] + matrix[5] * v3[2] - translate[1]);
    output3[2] += weight * (matrix[6] * v3[0] + matrix[7] * v3[1] + matrix[8] * v3[2] - translate[2]);

    //output3[0] += weight * (v3[0] + joint_pos[0]);
    //output3[1] += weight * (v3[1] + joint_pos[1]);
    //output3[2] += weight * (v3[2] + joint_pos[2]);
}

void OssParser::poseIt(unsigned int pose)
{
    float* output = new float[num_vertices * 3];
    for (unsigned int i = 0; i < num_vertices; ++i) {
        output[3 * i] = 0;
        output[3 * i + 1] = 0;
        output[3 * i + 2] = 0;
        for (unsigned int j = 0; j < weights[i].weights.size(); ++j) {
            unsigned int joint = weights[i].weights[j].joint;
            float weight = weights[i].weights[j].weight;
            apply_tranform(&output[3 * i], &vertices[3 * i], &joints[pose * num_joints * 7 + joint * 7], &joints[joint*7], weight);
        }
    }
    memcpy(vertices, output, num_vertices * 3 * sizeof(float));
    delete[] output;
}

int main()
{
    if constexpr (std::endian::native == std::endian::little)
        std::cout << "little-endian";
    else if constexpr (std::endian::native == std::endian::big)
        std::cout << "big-endian";
    else
        std::cout << "mixed-endian";
    printf("\n%d\n", pair<int, int>(1, 1)<pair<int, int>(1, 2));
    vector<string> filenames;
    // = 
    //{
    //    "burglar"
    //};
    std::string path = "./oss/";
    std::string oss_ext = ".oss";
    for (const auto& file : fs::directory_iterator(path)) {
        if (file.path().extension() == oss_ext) {
            std::cout << file.path().stem() << std::endl;
            filenames.push_back(file.path().stem().string());
        }
    }
    unsigned int pos = 0;
    for (auto iter = filenames.begin(); iter != filenames.end(); iter++) {
        std::cout << *iter << std::endl;
        try {
            OssParser oss = OssParser("oss\\" + *iter + ".oss");
            // oss.poseIt(pos);
            oss.toGLTF(*iter + to_string(pos) + ".gltf", "oss\\" + *iter + ".png", pos);
        } catch (...) {
            std::cout << "Error!" << std::endl;
        }
    }
}