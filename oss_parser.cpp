#define _CRT_SECURE_NO_WARNINGS
#include <stdlib.h>
#include <crtdbg.h>

#include <iostream>
#include <string>
#include <vector>
#include <cstdio>
#include <cstring>
#include <bit>
#include <sstream>
#include <exception>

using namespace std;

#include <filesystem>
namespace fs = filesystem;

#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STBI_MSC_SECURE_CRT
#include "tiny_gltf.h"

///////////////////////////////////////////////////////////////////////////////////////////////////
// Вспомогательные функции
///////////////////////////////////////////////////////////////////////////////////////////////////

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
string toString(T val) {
    ostringstream oss;
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

///////////////////////////////////////////////////////////////////////////////////////////////////
// Вспомогательные структуры
///////////////////////////////////////////////////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////////////////////////////////////////////////
// Класс для разбора OSS файлов
///////////////////////////////////////////////////////////////////////////////////////////////////

class OssParser {
  public:
    OssParser(const string &filename);
    ~OssParser();
    // Функции
    void toGLTF(const string& filename, const string& texture);
    unsigned int add(tinygltf::Model& m, const string& texture);
    unsigned int getNumFrames();
    unsigned int getFramePerSecond();
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

///////////////////////////////////////////////////////////////////////////////////////////////////
// Реализация OssParser
///////////////////////////////////////////////////////////////////////////////////////////////////

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
OssParser::OssParser(const string& filename) {
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
        // printf("      NumWeights for vertex %d = %d; JointsInfo = [", i, num_weights);
        Weights joint_weight;
        for (unsigned int j = 0; j < num_weights; ++j) {
            fread(&weight.joint, sizeof(int), 1, f);
            fread(&weight.weight, sizeof(float), 1, f);
            // printf("(%d, %f)", weight.joint, weight.weight);
            if (j != num_weights - 1) {
                // printf(", ");
            }
            joint_weight.weights.push_back(weight);
        }
        // printf("]\n");
        weights[i] = joint_weight;
    }
    printf("    Position of Joints = %x\n", ftell(f));
    joints = new float[7 * num_joints * num_frames];
    fread(joints, sizeof(float), 7 * num_joints * num_frames, f);
    fclose(f);
}

OssParser::~OssParser() {
    delete[] vertices;
    delete[] triangles;
    delete[] uvs;
    delete[] uvis;
    delete[] joints;
    delete[] weights;
}

void OssParser::toGLTF(const string& filename, const string& texture) {
    tinygltf::Model m;      // Файл gltf
    tinygltf::Scene scene;  // Сцена

    // Непонятная штука, видимо задает фичи gltf
    m.asset.version = "2.0";
    m.asset.generator = "tinygltf";

    tinygltf::TinyGLTF gltf;
    scene.nodes.push_back(add(m, texture));
    m.scenes.push_back(scene);

    gltf.WriteGltfSceneToFile(&m, filename.c_str(),
                              true,    // embedImages
                              true,    // embedBuffers
                              true,    // pretty print
                              false);  // write binary
}

unsigned int OssParser::add(tinygltf::Model& m, const string& texture) {
    // Текущие индексы в модели
    unsigned int accessor_index = (unsigned)m.accessors.size();
    unsigned int buffer_view_index = (unsigned)m.bufferViews.size();
    unsigned int buffer_index = (unsigned)m.buffers.size();
    unsigned int material_index = (unsigned)m.materials.size();
    unsigned int node_index = (unsigned)m.nodes.size();
    unsigned int mesh_index = (unsigned)m.meshes.size();
    unsigned int skin_index = (unsigned)m.skins.size();
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // 1. Находим уникальные вертексы с уникальными uv координатами, я решил это сделать в map
    ///////////////////////////////////////////////////////////////////////////////////////////////
    unsigned int max_triangles_in_vertex = 0;
    map<pair<unsigned int, unsigned int>, unsigned int> v_map;
    for (unsigned int i = 0; i < 3 * num_triangles; ++i) {
        unsigned int vA = triangles[i];  // индекс вертекса треугольника
        unsigned int uvA = uvis[i];      // uv индекс вертекса треугольника
        auto f_iter = v_map.find({vA, uvA});
        if (f_iter != v_map.end()) {  // если в массиве уже есть даный вертекс с данным uv индексом
            f_iter->second += 1;      // считаем сколько таких вершин (в одной вершине могут сходиться несколько треугольников
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
    // 2. Заполняем новый массив вертексами из map и записать индекс в качестве значения map
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
            if (f_iter != s_map.end()) {      // если в массиве уже есть даный вертекс с данным uv индексом
                f_iter->second += j->weight;  // считаем сколько таких вершин (в одной вершине могут сходиться несколько треугольников
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
    // 3. Заполняем буфер индексов треугольников
    ///////////////////////////////////////////////////////////////////////////////////////////////
    for (unsigned int i = 0; i < 3 * num_triangles; ++i) {
        unsigned int vA = triangles[i];
        unsigned int uvA = uvis[i];
        f_triangles[i] = v_map[{vA, uvA}];
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // 4. Веса и индексы костей вертексов
    ///////////////////////////////////////////////////////////////////////////////////////////////
    unsigned int max_weights = 0;
    for (unsigned int i = 0; i < n_vertices; ++i) {
        if (max_weights < (unsigned int)f_weights[i].weights.size()) {
            max_weights = (unsigned int)f_weights[i].weights.size();
        }
    }
    unsigned int num_jw_arrays = (max_weights - 1) / 4 + 1;
    unsigned short** s_joints = new unsigned short*[num_jw_arrays];
    float** s_weights = new float*[num_jw_arrays];
    for (unsigned int i = 0; i < num_jw_arrays; ++i) {
        s_joints[i] = new unsigned short[4 * n_vertices];
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
            s_joints[j / 4][4 * i + j % 4] = (unsigned short)(f_weights[i].weights[j].joint);
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
    tinygltf::Mesh mesh;  // Меш в сцене
    // В каждом gltf mesh есть primitive (определяет как мы рисуем треугольники) он может быть и не один
    tinygltf::Primitive primitive;
    primitive.indices = accessor_index + 1;
    primitive.attributes["POSITION"] = accessor_index + 0;
    primitive.attributes["TEXCOORD_0"] = accessor_index + 2;
    string str_joints = "JOINTS_";
    string str_weights = "WEIGHTS_";
    for (unsigned i = 0; i < num_jw_arrays; ++i) {
        primitive.attributes[(str_joints + toString(i)).c_str()] = accessor_index + 3 + 2 * i;
        primitive.attributes[(str_weights + toString(i)).c_str()] = accessor_index + 3 + 2 * i + 1;
    }
    primitive.material = material_index + 0;
    primitive.mode = TINYGLTF_MODE_TRIANGLES;

    mesh.primitives.push_back(primitive);

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

    bv_vertices.buffer = buffer_index + 0;
    bv_vertices.byteOffset = 0;
    bv_vertices.byteLength = n_vertices * 3 * sizeof(float);
    bv_vertices.target = TINYGLTF_TARGET_ARRAY_BUFFER;

    a_vertices.bufferView = buffer_view_index + 0;
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

    bv_triangles.buffer = buffer_index + 1;
    bv_triangles.byteOffset = 0;
    bv_triangles.byteLength = num_triangles * 3 * sizeof(unsigned int);
    bv_triangles.target = TINYGLTF_TARGET_ELEMENT_ARRAY_BUFFER;

    a_triangles.bufferView = buffer_view_index + 1;
    a_triangles.byteOffset = 0;
    a_triangles.componentType = TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT;
    a_triangles.count = num_triangles * 3;
    a_triangles.type = TINYGLTF_TYPE_SCALAR;
    a_triangles.maxValues.push_back(n_vertices - 1);
    a_triangles.minValues.push_back(0);

    bv_uvs.buffer = buffer_index + 2;
    bv_uvs.byteOffset = 0;
    bv_uvs.byteLength = n_vertices * 2 * sizeof(float);
    bv_uvs.target = TINYGLTF_TARGET_ARRAY_BUFFER;

    a_uvs.bufferView = buffer_view_index + 2;
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
    vector<tinygltf::Buffer> b_joint_weights;
    vector<tinygltf::BufferView> bv_weights;
    vector<tinygltf::BufferView> bv_joints;
    vector<tinygltf::Accessor> a_weights;
    vector<tinygltf::Accessor> a_joints;

    for (unsigned i = 0; i < num_jw_arrays; ++i) {
        // Заполняем буферы
        tinygltf::Buffer b_joint_weight;
        b_joint_weight.data.resize(n_vertices * 4 * sizeof(unsigned short) + n_vertices * 4 * sizeof(float));
        memcpy(&b_joint_weight.data[0], &s_joints[i][0], n_vertices * 4 * sizeof(unsigned short));
        memcpy(&b_joint_weight.data[n_vertices * 4 * sizeof(unsigned short)], &s_weights[i][0], n_vertices * 4 * sizeof(float));

        b_joint_weights.push_back(b_joint_weight);

        // Говорим программе, где искать данные
        tinygltf::BufferView bv_joint;
        tinygltf::BufferView bv_weight;
        bv_joint.buffer = buffer_index + i + 3;
        bv_joint.byteOffset = 0;
        bv_joint.byteLength = n_vertices * 4 * sizeof(unsigned short);
        bv_joint.target = TINYGLTF_TARGET_ARRAY_BUFFER;

        bv_weight.buffer = buffer_index + i + 3;
        bv_weight.byteOffset = n_vertices * 4 * sizeof(unsigned short);
        bv_weight.byteLength = n_vertices * 4 * sizeof(float);
        bv_weight.target = TINYGLTF_TARGET_ARRAY_BUFFER;

        bv_joints.push_back(bv_joint);
        bv_weights.push_back(bv_weight);

        // Объясняем, как их просматривать
        tinygltf::Accessor a_joint;
        tinygltf::Accessor a_weight;

        a_joint.bufferView = buffer_view_index + 3 + 2 * i;
        a_joint.byteOffset = 0;
        a_joint.componentType = TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT;
        a_joint.count = n_vertices;
        a_joint.type = TINYGLTF_TYPE_VEC4;
        double xmin = (double)min_array<unsigned short>(s_joints[i], 0, 4, n_vertices * 4);
        double ymin = (double)min_array<unsigned short>(s_joints[i], 1, 4, n_vertices * 4);
        double zmin = (double)min_array<unsigned short>(s_joints[i], 2, 4, n_vertices * 4);
        double tmin = (double)min_array<unsigned short>(s_joints[i], 3, 4, n_vertices * 4);
        double xmax = (double)max_array<unsigned short>(s_joints[i], 0, 4, n_vertices * 4);
        double ymax = (double)max_array<unsigned short>(s_joints[i], 1, 4, n_vertices * 4);
        double zmax = (double)max_array<unsigned short>(s_joints[i], 2, 4, n_vertices * 4);
        double tmax = (double)max_array<unsigned short>(s_joints[i], 3, 4, n_vertices * 4);
        a_joint.maxValues = {xmax, ymax, zmax, tmax};
        a_joint.minValues = {xmin, ymin, zmin, tmin};

        a_weight.bufferView = buffer_view_index + 3 + 2 * i + 1;  // 3 + 2 * (n_jw - 1) + 1 = 2 * n_jw +2
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

    bv_times.buffer = buffer_index + num_jw_arrays + 3;
    bv_times.byteOffset = 0;
    bv_times.byteLength = num_frames * sizeof(float);

    a_times.bufferView = buffer_view_index + 2 * num_jw_arrays + 3;
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
    vector<tinygltf::BufferView> bv_anims_rotations;
    vector<tinygltf::BufferView> bv_anims_translations;
    vector<tinygltf::Accessor> a_anims_rotations;
    vector<tinygltf::Accessor> a_anims_translations;

    // buffer будет общий (Общий нельзя! Ошибка - bufferView.byteStride must not be defined for buffer views used by animation sampler accessors.)
    float* translations = new float[3 * num_joints * num_frames];
    float* rotations = new float[4 * num_joints * num_frames];
    for (unsigned i = 0; i < num_joints * num_frames; ++i) {
        translations[3 * i] = joints[7 * i];
        translations[3 * i + 1] = joints[7 * i + 1];
        translations[3 * i + 2] = joints[7 * i + 2];
        float quat2 = powf(joints[7 * i + 3], 2) + powf(joints[7 * i + 4], 2) + powf(joints[7 * i + 5], 2) + powf(joints[7 * i + 6], 2);
        float quat = sqrtf(quat2);
        rotations[4 * i] = -joints[7 * i + 3] / quat;
        rotations[4 * i + 1] = -joints[7 * i + 4] / quat;
        rotations[4 * i + 2] = -joints[7 * i + 5] / quat;
        rotations[4 * i + 3] = joints[7 * i + 6] / quat;
    }
    b_anims.data.resize(7 * num_joints * num_frames * sizeof(float));
    memcpy(&b_anims.data[0], &translations[0], 3 * num_joints * num_frames * sizeof(float));
    memcpy(&b_anims.data[3 * num_joints * num_frames * sizeof(float)], &rotations[0], 4 * num_joints * num_frames * sizeof(float));
    m.buffers.push_back(b_anims);

    delete[] translations;
    delete[] rotations;

    // Проходимся по костям и создаём для каждой кости bufferView и accessor отдельно для
    // трансляций, отдельно для кватернионов
    tinygltf::Animation anim;
    if (m.animations.size() != 0) {
        anim = m.animations[0];
    }
    unsigned index_buffer = (unsigned)m.buffers.size() - 1;
    unsigned index_times = (unsigned)m.accessors.size() - 1;
    for (unsigned i = 0; i < num_joints; ++i) {
        unsigned index_buffer_view = (unsigned)m.bufferViews.size();
        unsigned index_accessor = (unsigned)m.accessors.size();
        unsigned index_sampler = (unsigned)anim.samplers.size();

        tinygltf::BufferView bv_anims_rotation;
        tinygltf::BufferView bv_anims_translation;

        bv_anims_translation.buffer = index_buffer;
        bv_anims_translation.byteOffset = 3 * i * num_frames * sizeof(float);
        bv_anims_translation.byteLength = 3 * num_frames * sizeof(float);

        bv_anims_rotation.buffer = index_buffer;
        bv_anims_rotation.byteOffset = 3 * num_joints * num_frames * sizeof(float) + 4 * i * num_frames * sizeof(float);
        bv_anims_rotation.byteLength = 4 * num_frames * sizeof(float);

        m.bufferViews.push_back(bv_anims_translation);
        m.bufferViews.push_back(bv_anims_rotation);

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

        m.accessors.push_back(a_anims_translation);
        m.accessors.push_back(a_anims_rotation);

        // Готовим анимации - samler - безье кривая, от времени
        tinygltf::AnimationSampler anim_smp1;
        anim_smp1.interpolation = "LINEAR";
        anim_smp1.input = index_times;      // accessor to times
        anim_smp1.output = index_accessor;  // accessor to translations

        tinygltf::AnimationSampler anim_smp2;
        anim_smp2.interpolation = "LINEAR";
        anim_smp2.input = index_times;          // accessor to times
        anim_smp2.output = index_accessor + 1;  // accessor to rotations

        anim.samplers.push_back(anim_smp1);
        anim.samplers.push_back(anim_smp2);

        tinygltf::AnimationChannel anim_chan1;
        anim_chan1.sampler = index_sampler;
        anim_chan1.target_node = node_index + i;
        anim_chan1.target_path = "translation";

        tinygltf::AnimationChannel anim_chan2;
        anim_chan2.sampler = index_sampler + 1;
        anim_chan2.target_node = node_index + i;
        anim_chan2.target_path = "rotation";

        anim.channels.push_back(anim_chan1);
        anim.channels.push_back(anim_chan2);
    }

    // printf("target_node = %d\n", anim.channels[0].target_node);
    if (m.animations.size() == 0) {
        m.animations.push_back(anim);
    } else {
        m.animations[0] = anim;
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Отдельная проблема это InverseBindMatrices
    // К сожалению, но код ниже работает неправильно - иногда det матрицы равен 0. Обращение даёт
    // бред, к счастью, как оказалось она и не нужна! Вот только для assimp она нужна...
    ///////////////////////////////////////////////////////////////////////////////////////////////
    tinygltf::Buffer ibm_buffer;
    tinygltf::BufferView ibm_buffer_view;
    tinygltf::Accessor ibm_accessor;
    
    float* ibm = new float[16 * num_joints];
    float inv_ibm[16] = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    };
    //float identity[7] = {0, 0, 0, 0, 0, 0, 1};
    for (unsigned int i = 0; i<num_joints; ++i) {
        memcpy(&ibm[16*i], &inv_ibm[0], 16 * sizeof(float));
    }
    ibm_buffer.data.resize(16 * num_joints * sizeof(float));
    memcpy(&ibm_buffer.data[0], &ibm[0], 16 * num_joints * sizeof(float));
    m.buffers.push_back(ibm_buffer);
    
    ibm_buffer_view.buffer = (unsigned) m.buffers.size() - 1;
    ibm_buffer_view.byteOffset = 0;
    ibm_buffer_view.byteLength = 16 * num_joints * sizeof(float);
    m.bufferViews.push_back(ibm_buffer_view);
    
    ibm_accessor.bufferView = (unsigned) m.bufferViews.size() - 1;
    ibm_accessor.byteOffset = 0;
    ibm_accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
    ibm_accessor.count = num_joints;
    ibm_accessor.type = TINYGLTF_TYPE_MAT4;
    m.accessors.push_back(ibm_accessor);
    
    unsigned ibm_accessor_index = (unsigned) m.accessors.size() - 1;
    delete [] ibm;
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Скелет
    ///////////////////////////////////////////////////////////////////////////////////////////////

    tinygltf::Node main_node;  // Структура скелета передаётся в node (пока не используется)
    main_node.mesh = mesh_index;
    main_node.skin = skin_index;
    char strbuf[1024];
    sprintf(strbuf, "Main.%03d", node_index + num_joints);
    main_node.name = string(strbuf);

    for (unsigned int i = 0; i < num_joints; ++i) {
        tinygltf::Node node;  // Структура скелета передаётся в node (пока не используется)
        node.rotation = {0.0f, 0.0f, 0.0f, 1.0f};
        //{
        //    joints[num_frames * 7 * i + 3],
        //    joints[num_frames * 7 * i + 4],
        //    joints[num_frames * 7 * i + 5],
        //    joints[num_frames * 7 * i + 6]
        //};
        node.translation =  //{ center_masses[3 * i], center_masses[3 * i + 1], center_masses[3 * i + 2] };
            {0.0f, 0.0f, 0.0f};
        //{
        //    joints[num_frames * 7 * i + 0],
        //    joints[num_frames * 7 * i + 1],
        //    joints[num_frames * 7 * i + 2]
        //};
        sprintf(strbuf, "Joint.%03d", i);
        node.name = string(strbuf);
        main_node.children.push_back(node_index + i);
        m.nodes.push_back(node);
    }

    m.nodes.push_back(main_node);

    tinygltf::Skin skin;
    for (unsigned int i = 0; i < num_joints; ++i) {
        skin.joints.push_back(node_index + i);
    }
    skin.inverseBindMatrices = ibm_accessor_index;
    m.skins.push_back(skin);

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Материалы
    ///////////////////////////////////////////////////////////////////////////////////////////////

    tinygltf::Material mat;
    mat.doubleSided = true;
    unsigned int img_color_src = (unsigned) m.images.size();
    unsigned int img_normal_src = (unsigned) m.images.size() + 1;
    for (unsigned int i = 0; i < (unsigned) m.images.size(); ++i) {
        if (m.images[i].uri == texture + ".dds") {
            img_color_src = i;
        }
        if (m.images[i].uri == texture + "_NRM.dds") {
            img_normal_src = i;
        }
    }

    if (fs::exists(texture + ".dds")) {
        tinygltf::Texture tex_color;
        tex_color.sampler = 0;
        tex_color.source = img_color_src;
        m.textures.push_back(tex_color);

        if (img_color_src == m.images.size()) {
            tinygltf::Image img_color;
            img_color.uri = texture + ".dds";
            m.images.push_back(img_color);
        }
        mat.pbrMetallicRoughness.baseColorTexture.index = (unsigned) m.textures.size() - 1;
    }

    if (fs::exists(texture + "_NRM.dds")) {
        tinygltf::Texture tex_normal;
        tex_normal.sampler = 0;
        tex_normal.source = img_normal_src;
        m.textures.push_back(tex_normal);

        if (img_normal_src == m.images.size()) {
            tinygltf::Image img_color;
            img_color.uri = texture + "_NRM.dds";
            m.images.push_back(img_color);
        }
        mat.normalTexture.index = (unsigned) m.textures.size() - 1;
    }

    m.materials.push_back(mat);

    if (m.samplers.size() == 0) {
        tinygltf::Sampler smp;
        smp.magFilter = TINYGLTF_TEXTURE_FILTER_LINEAR;
        smp.minFilter = TINYGLTF_TEXTURE_FILTER_LINEAR_MIPMAP_LINEAR;
        smp.wrapS = TINYGLTF_TEXTURE_WRAP_REPEAT;
        smp.wrapT = TINYGLTF_TEXTURE_WRAP_REPEAT;
        m.samplers.push_back(smp);
    }

    delete[] f_triangles;
    delete[] f_vertices;
    delete[] f_uvs;
    for (unsigned int i = 0; i < num_jw_arrays; ++i) {
        delete[] s_joints[i];
        delete[] s_weights[i];
    }
    delete[] s_joints;
    delete[] s_weights;
    delete[] times;

    return (unsigned)m.nodes.size() - 1;
}

unsigned int OssParser::getNumFrames() { 
    return num_frames; 
}

unsigned int OssParser::getFramePerSecond() {
    return frame_per_second; 
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Класс для разбора OSM файлов
//////////////////////////////////////////////////////////////////////////////////////////////////

class OsmParser {
  public:
    OsmParser(const string &filename);
    ~OsmParser();
    // Функции
    void toGLTF(const string& filename, const string& texture);
    unsigned int add(tinygltf::Model& m, const string& texture, const string& tlfname, unsigned int frame_per_second, unsigned int num_frames);
  private:
    char idp2[4];
    unsigned int unknown1;      // 8
    unsigned int unknown2;      // 256
    unsigned int unknown3;      // 256
    unsigned int frame_size;    // разные 1392 (archer_dagger) 1696 (assasin_knife) - размер блока с точками 3d
    unsigned int unknown5;      // 1
    unsigned int num_vertices;  // разные 86 (archer_dagger) 105 (assasin_knife)
    unsigned int num_uvs;       // разные 88 (archer_dagger) 234 (assasin_knife) - количество точек в uv_map
    unsigned int num_triangles; // разные 70 (archer_dagger) 88 (assasin_knife)
    unsigned int unknown9;      // 0
    unsigned int unknown10;     // 1
    unsigned int point_skin_bit_map;  // разные 68 (archer_dagger) 68 (assasin_knife) - адрес skin_bit_map
    unsigned int point_uv;            // разные 132 (archer_dagger) 132 (assasin_knife) - адрес блока uv_map - точек float[unknown7][2]
    unsigned int point_uvi_faces;     // разные 836 (archer_dagger) 2004 (assasin_knife) - адрес блока с целыми числами unsigned int[unknown8][3][2]
    unsigned int point_frame;         // разные 2516 (archer_dagger) 4116 (assasin_knife) - адрес FRAME 16 байт на строку и затем Vertex[unknown6]
    unsigned int point_end_frame;     // разные 3908 (archer_dagger) 5812 (assasin_knife) - размер файла
    unsigned int point_end_file;      // разные 3908 (archer_dagger) 5812 (assasin_knife)- размер файла
    char skin_bit_map[64];
    float* uvs;
    unsigned int* triangles;
    unsigned int* uvis;
    char frame[16];          // "FRAME 000        "
    float* vertices;
    unsigned int* unknown_array;
};

///////////////////////////////////////////////////////////////////////////////////////////////////
// Реализация OsmParser
///////////////////////////////////////////////////////////////////////////////////////////////////

OsmParser::OsmParser(const string& filename) {
    FILE* f = fopen(filename.c_str(), "rb");
    fread(&idp2[0], sizeof(char), 4, f);
    fread(&unknown1, sizeof(int), 1, f);
    fread(&unknown2, sizeof(int), 1, f);
    fread(&unknown3, sizeof(int), 1, f);
    fread(&frame_size, sizeof(int), 1, f);
    fread(&unknown5, sizeof(int), 1, f);
    fread(&num_vertices, sizeof(int), 1, f);
    fread(&num_uvs, sizeof(int), 1, f);
    fread(&num_triangles, sizeof(int), 1, f);
    fread(&unknown9, sizeof(int), 1, f);
    fread(&unknown10, sizeof(int), 1, f);
    fread(&point_skin_bit_map, sizeof(int), 1, f);
    fread(&point_uv          , sizeof(int), 1, f);
    fread(&point_uvi_faces   , sizeof(int), 1, f);
    fread(&point_frame       , sizeof(int), 1, f);
    fread(&point_end_frame   , sizeof(int), 1, f);
    fread(&point_end_file    , sizeof(int), 1, f);
    printf("Number of vertices = %d\n", num_vertices);
    printf("Number of triangles = %d\n", num_triangles);
    printf("Number of uvs = %d\n", num_uvs);
    printf("    Position of UVs = %x\n", point_uv);
    printf("    Position of Triangles and Uvis = %x\n", point_uvi_faces);
    printf("    Position of Vertices = %x\n", point_frame + 16);
    fread(&skin_bit_map[0], sizeof(char), 64, f);
    uvs = new float[2 * num_uvs];
    fread(&uvs[0], sizeof(float), 2 * num_uvs, f);
    ///////////////////////////////////////////////////////////////////////////////////////////////
    triangles = new unsigned int[3 * num_triangles];
    uvis = new unsigned int[3 * num_triangles];
    unsigned int* triangles_uvis = new unsigned int[6 * num_triangles];
    fread(&triangles_uvis[0], sizeof(float), 6 * num_triangles, f); // Сначала три точки - треугольник, затем треугольник в uv
    for (unsigned int i = 0; i < 3 * num_triangles; ++i) {
        // 0 -> 0
        // 1 -> 1
        // 2 -> 2
        // 3 -> 6 6 * (i / 3) + i % 3
        // 4 -> 7
        // 5 -> 8
        // 6 -> 12
        // 7 -> 13
        // 8 -> 14
        // 9 -> 18
        // 10 -> 19
        // 11 -> 20
        triangles[i] = triangles_uvis[6 * (i / 3) + i % 3];
        uvis[i] = triangles_uvis[6 * (i / 3) + 3 + i % 3];
    }
    delete [] triangles_uvis;
    fread(&frame[0], sizeof(char), 16, f);
    vertices = new float[3 * num_vertices];
    unknown_array = new unsigned int[num_vertices];
    for (unsigned int i = 0; i < num_vertices; ++i) {
        fread(&vertices[3 * i], sizeof(float), 3, f);
        fread(&unknown_array[i], sizeof(int), 1, f);
    }
    // Похоже они объединили joint и вертексы в одном месте (морфирующая анимация?) Надо искать что-то с 2 фреймами
    fclose(f);
}

OsmParser::~OsmParser() {
    delete [] uvs;
    delete [] triangles;
    delete [] uvis;
    delete [] vertices;
    delete [] unknown_array;
}

void OsmParser::toGLTF(const string& filename, const string& texture) {
    tinygltf::Model m;      // Файл gltf
    tinygltf::Scene scene;  // Сцена

    // Непонятная штука, видимо задает фичи gltf
    m.asset.version = "2.0";
    m.asset.generator = "tinygltf";

    tinygltf::TinyGLTF gltf;
    scene.nodes.push_back(add(m, texture, "", 0, 0));
    m.scenes.push_back(scene);

    gltf.WriteGltfSceneToFile(&m, filename.c_str(),
                              true,    // embedImages
                              true,    // embedBuffers
                              true,    // pretty print
                              false);  // write binary
}

unsigned int OsmParser::add(tinygltf::Model& m, const string& texture, const string& tlfname, unsigned int frame_per_second, unsigned int num_frames) {
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // 1. Находим уникальные вертексы с уникальными uv координатами, я решил это сделать в map
    ///////////////////////////////////////////////////////////////////////////////////////////////
    unsigned int max_triangles_in_vertex = 0;
    map<pair<unsigned int, unsigned int>, unsigned int> v_map;
    for (unsigned int i = 0; i < 3 * num_triangles; ++i) {
        unsigned int vA = triangles[i];  // индекс вертекса треугольника
        unsigned int uvA = uvis[i];      // uv индекс вертекса треугольника
        auto f_iter = v_map.find({vA, uvA});
        if (f_iter != v_map.end()) {  // если в массиве уже есть даный вертекс с данным uv индексом
            f_iter->second += 1;      // считаем сколько таких вершин (в одной вершине могут сходиться несколько треугольников
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
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // 2. Заполняем новый массив вертексами из map и записать индекс в качестве значения map
    ///////////////////////////////////////////////////////////////////////////////////////////////
    unsigned int ind = 0;
    for (auto i = v_map.begin(); i != v_map.end(); i++) {
        f_vertices[3 * ind] = vertices[3 * i->first.first];
        f_vertices[3 * ind + 1] = vertices[3 * i->first.first + 1];
        f_vertices[3 * ind + 2] = vertices[3 * i->first.first + 2];
        f_uvs[2 * ind] = uvs[2 * i->first.second];
        f_uvs[2 * ind + 1] = 1 - uvs[2 * i->first.second + 1];
        i->second = ind;  // заменяем в v_map количество треугольников, которые сходятся в данном вертексе на индекс в
                          // v_map, чтобы потом не бегать по v_map, когда будем переопределять треугольники
        ++ind;
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // 3. Заполняем буфер индексов треугольников
    ///////////////////////////////////////////////////////////////////////////////////////////////
    for (unsigned int i = 0; i < 3 * num_triangles; ++i) {
        unsigned int vA = triangles[i];
        unsigned int uvA = uvis[i];
        f_triangles[i] = v_map[{vA, uvA}];
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Блок вывода информации о подготовленных данных
    ////////////////////////////////////////////////////////////////////////////////////////////////
    printf("n_vertices size = %d\n", n_vertices);
    printf("f_vertices size = %d\n", n_vertices * 3);
    printf("f_triangles size = %d\n", num_triangles * 3);
    printf("f_uvs size = %d\n", n_vertices * 2);
    printf("Size of map = %lld sizeof(float) = %lld\n", v_map.size(), sizeof(float));
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Подготовка данных завершена, можно заполнять данные gltf
    ////////////////////////////////////////////////////////////////////////////////////////////////
    tinygltf::Mesh mesh;  // Меш в сцене
    // Текущие индексы в модели
    unsigned int accessor_index = (unsigned)m.accessors.size();
    unsigned int buffer_view_index = (unsigned)m.bufferViews.size();
    unsigned int buffer_index = (unsigned)m.buffers.size();
    unsigned int material_index = (unsigned)m.materials.size();
    unsigned int node_index = (unsigned)m.nodes.size();
    unsigned int mesh_index = (unsigned)m.meshes.size();
    // В каждом gltf mesh есть primitive (определяет как мы рисуем треугольники) он может быть и не один
    tinygltf::Primitive primitive;
    primitive.indices = accessor_index + 1;
    primitive.attributes["POSITION"] = accessor_index + 0;
    primitive.attributes["TEXCOORD_0"] = accessor_index + 2;
    primitive.material = material_index + 0;
    primitive.mode = TINYGLTF_MODE_TRIANGLES;

    mesh.primitives.push_back(primitive);

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

    bv_vertices.buffer = buffer_index + 0;
    bv_vertices.byteOffset = 0;
    bv_vertices.byteLength = n_vertices * 3 * sizeof(float);
    bv_vertices.target = TINYGLTF_TARGET_ARRAY_BUFFER;

    a_vertices.bufferView = buffer_view_index + 0;
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

    bv_triangles.buffer = buffer_index + 1;
    bv_triangles.byteOffset = 0;
    bv_triangles.byteLength = num_triangles * 3 * sizeof(unsigned int);
    bv_triangles.target = TINYGLTF_TARGET_ELEMENT_ARRAY_BUFFER;

    a_triangles.bufferView = buffer_view_index + 1;
    a_triangles.byteOffset = 0;
    a_triangles.componentType = TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT;
    a_triangles.count = num_triangles * 3;
    a_triangles.type = TINYGLTF_TYPE_SCALAR;
    a_triangles.maxValues.push_back(n_vertices - 1);
    a_triangles.minValues.push_back(0);

    bv_uvs.buffer = buffer_index + 2;
    bv_uvs.byteOffset = 0;
    bv_uvs.byteLength = n_vertices * 2 * sizeof(float);
    bv_uvs.target = TINYGLTF_TARGET_ARRAY_BUFFER;

    a_uvs.bufferView = buffer_view_index + 2;
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

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Буферы анимации
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Анимация в osm реализцется через tlf файлы
    ///////////////////////////////////////////////////////////////////////////////////////////////
    if (num_frames != 0 && fs::exists(tlfname) && frame_per_second != 0) {
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

        bv_times.buffer = buffer_index + 3;
        bv_times.byteOffset = 0;
        bv_times.byteLength = num_frames * sizeof(float);

        a_times.bufferView = buffer_view_index + 3;
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
        tinygltf::BufferView bv_anims_rotations;
        tinygltf::BufferView bv_anims_translations;
        tinygltf::Accessor a_anims_rotations;
        tinygltf::Accessor a_anims_translations;

        // buffer будет общий (Общий нельзя! Ошибка - bufferView.byteStride must not be defined for buffer views used by animation sampler accessors.)
        float* translations = new float[3 * num_frames];
        float* rotations = new float[4 * num_frames];
        FILE* tlf = fopen(tlfname.c_str(), "r");
        if (tlf != NULL) {
            for (unsigned i = 0; i < num_frames; ++i) {
                fscanf(tlf, "%f %f %f", &translations[3 * i], &translations[3 * i + 1], &translations[3 * i + 2]);
                fscanf(tlf, "%f %f %f %f", &rotations[4 * i], &rotations[4 * i + 1], &rotations[4 * i + 2], &rotations[4 * i + 3]);
                float quat2 = powf(rotations[4 * i], 2) + powf(rotations[4 * i + 1], 2) + powf(rotations[4 * i + 2], 2) + powf(rotations[4 * i + 3], 2);
                float quat = sqrtf(quat2);
                rotations[4 * i] = - rotations[4 * i] / quat;
                rotations[4 * i + 1] = - rotations[4 * i + 1] / quat;
                rotations[4 * i + 2] = - rotations[4 * i + 2] / quat;
                rotations[4 * i + 3] = rotations[4 * i + 3] / quat;
            }
        }
        fclose(tlf);
        b_anims.data.resize(7 * num_frames * sizeof(float));
        memcpy(&b_anims.data[0], &translations[0], 3 * num_frames * sizeof(float));
        memcpy(&b_anims.data[3 * num_frames * sizeof(float)], &rotations[0], 4 * num_frames * sizeof(float));
        m.buffers.push_back(b_anims);

        delete[] translations;
        delete[] rotations;

        // Проходимся по костям и создаём для каждой кости bufferView и accessor отдельно для
        // трансляций, отдельно для кватернионов
        tinygltf::Animation anim;
        if (m.animations.size() != 0) {
            anim = m.animations[0];
        }
        unsigned index_buffer = (unsigned)m.buffers.size() - 1;
        unsigned index_times = (unsigned)m.accessors.size() - 1;
        unsigned index_buffer_view = (unsigned)m.bufferViews.size();
        unsigned index_accessor = (unsigned)m.accessors.size();
        unsigned index_sampler = (unsigned)anim.samplers.size();

        tinygltf::BufferView bv_anims_rotation;
        tinygltf::BufferView bv_anims_translation;

        bv_anims_translation.buffer = index_buffer;
        bv_anims_translation.byteOffset = 0;
        bv_anims_translation.byteLength = 3 * num_frames * sizeof(float);

        bv_anims_rotation.buffer = index_buffer;
        bv_anims_rotation.byteOffset = 3 * num_frames * sizeof(float);
        bv_anims_rotation.byteLength = 4 * num_frames * sizeof(float);

        m.bufferViews.push_back(bv_anims_translation);
        m.bufferViews.push_back(bv_anims_rotation);

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

        m.accessors.push_back(a_anims_translation);
        m.accessors.push_back(a_anims_rotation);

        // Готовим анимации - samler - безье кривая, от времени
        tinygltf::AnimationSampler anim_smp1;
        anim_smp1.interpolation = "LINEAR";
        anim_smp1.input = index_times;      // accessor to times
        anim_smp1.output = index_accessor;  // accessor to translations

        tinygltf::AnimationSampler anim_smp2;
        anim_smp2.interpolation = "LINEAR";
        anim_smp2.input = index_times;          // accessor to times
        anim_smp2.output = index_accessor + 1;  // accessor to rotations

        anim.samplers.push_back(anim_smp1);
        anim.samplers.push_back(anim_smp2);

        tinygltf::AnimationChannel anim_chan1;
        anim_chan1.sampler = index_sampler;
        anim_chan1.target_node = node_index;
        anim_chan1.target_path = "translation";

        tinygltf::AnimationChannel anim_chan2;
        anim_chan2.sampler = index_sampler + 1;
        anim_chan2.target_node = node_index;
        anim_chan2.target_path = "rotation";

        anim.channels.push_back(anim_chan1);
        anim.channels.push_back(anim_chan2);

        // printf("target_node = %d\n", anim.channels[0].target_node);
        if (m.animations.size() == 0) {
            m.animations.push_back(anim);
        } else {
            m.animations[0] = anim;
        }
        
        delete[] times;
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Скелет
    ///////////////////////////////////////////////////////////////////////////////////////////////

    tinygltf::Node main_node;  // Структура скелета передаётся в node (пока не используется)
    main_node.mesh = mesh_index;
    char strbuf[1024];
    sprintf(strbuf, "Main.%03d", node_index);
    main_node.name = string(strbuf);

    m.nodes.push_back(main_node);

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Материалы
    ///////////////////////////////////////////////////////////////////////////////////////////////

    tinygltf::Material mat;
    mat.doubleSided = true;
    unsigned int img_color_src = (unsigned)m.images.size();
    unsigned int img_normal_src = (unsigned)m.images.size() + 1;
    for (unsigned int i = 0; i < (unsigned)m.images.size(); ++i) {
        if (m.images[i].uri == texture + ".dds") {
            img_color_src = i;
        }
        if (m.images[i].uri == texture + "_NRM.dds") {
            img_normal_src = i;
        }
    }

    if (fs::exists(texture + ".dds")) {
        tinygltf::Texture tex_color;
        tex_color.sampler = 0;
        tex_color.source = img_color_src;
        m.textures.push_back(tex_color);

        if (img_color_src == m.images.size()) {
            tinygltf::Image img_color;
            img_color.uri = texture + ".dds";
            m.images.push_back(img_color);
        }
        mat.pbrMetallicRoughness.baseColorTexture.index = (unsigned)m.textures.size() - 1;
    }

    if (fs::exists(texture + "_NRM.dds")) {
        tinygltf::Texture tex_normal;
        tex_normal.sampler = 0;
        tex_normal.source = img_normal_src;
        m.textures.push_back(tex_normal);

        if (img_normal_src == m.images.size()) {
            tinygltf::Image img_color;
            img_color.uri = texture + "_NRM.dds";
            m.images.push_back(img_color);
        }
        mat.normalTexture.index = (unsigned)m.textures.size() - 1;
    }

    m.materials.push_back(mat);

    if (m.samplers.size() == 0) {
        tinygltf::Sampler smp;
        smp.magFilter = TINYGLTF_TEXTURE_FILTER_LINEAR;
        smp.minFilter = TINYGLTF_TEXTURE_FILTER_LINEAR_MIPMAP_LINEAR;
        smp.wrapS = TINYGLTF_TEXTURE_WRAP_REPEAT;
        smp.wrapT = TINYGLTF_TEXTURE_WRAP_REPEAT;
        m.samplers.push_back(smp);
    }

    delete[] f_triangles;
    delete[] f_vertices;
    delete[] f_uvs;

    return (unsigned)m.nodes.size() - 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Функция, генерирующая сцены из нескольких фалов с attach
///////////////////////////////////////////////////////////////////////////////////////////////////

void genOSSwithAttach(const string &filename) {
    tinygltf::Model m;
    // Непонятная штука, видимо задает фичи gltf
    tinygltf::Asset asset;
    asset.version = "2.0";
    asset.generator = "tinygltf";
    m.asset = asset;

    fs::path main_file = filename;
    string main_dir    = main_file.parent_path().string();     // Папка
    string main_fname  = main_file.filename().stem().string(); // Имя файла без расширения и папки
    string dds_ext = ".dds";
    map<string, string> dds_map;
    for (const auto& file : fs::directory_iterator(main_dir + "/" )) {
        if (file.path().extension() == dds_ext) {
            cout << file.path().string() << endl;
            dds_map.insert({file.path().filename().stem().string(), string("oss/") + file.path().stem().string()});
        }
    }
    string oss_ext = ".oss";
    string osm_ext = ".osm";
    string main_texture = main_dir + "/" + main_fname;
    tinygltf::Node root_node;
    if (fs::exists(filename)) {
        OssParser oss = OssParser(filename);
        root_node.children.push_back(oss.add(m, main_texture));
        string attach_dir = main_dir + "/" + main_fname + ".attach/";
        if (fs::exists(attach_dir)) {
            for (const auto& file : fs::directory_iterator(attach_dir)) {
                string attachname = file.path().filename().stem().string();
                cout << attachname << endl;
                if (file.path().extension() == oss_ext) {
                    OssParser* oss2 = new OssParser(file.path().string());
                    if (dds_map.find(attachname) != dds_map.end()) {
                        cout << "Texture: " << dds_map[attachname] << endl;
                        root_node.children.push_back(oss2->add(m, dds_map[attachname]));
                    } else {
                        cout << "Texture: " << main_texture << endl;
                        root_node.children.push_back(oss2->add(m, main_texture));
                    }
                    delete oss2;
                }
                if (file.path().extension() == osm_ext) {
                    OsmParser* osm2 = new OsmParser(file.path().string());
                    if (dds_map.find(attachname) != dds_map.end()) {
                        cout << "Texture: " << dds_map[attachname] << endl;
                        root_node.children.push_back(osm2->add(m, dds_map[attachname], attach_dir + attachname + ".tlf", oss.getFramePerSecond(), oss.getNumFrames()));
                    } else {
                        cout << "Texture: " << main_texture << endl;
                        root_node.children.push_back(osm2->add(m, main_texture, attach_dir + attachname + ".tlf", oss.getFramePerSecond(), oss.getNumFrames()));
                    }
                    delete osm2;
                }
            }
        }
    }
    root_node.name = "Root";
    m.nodes.push_back(root_node);
    tinygltf::Scene scene;  // Сцена в файле
    scene.nodes.push_back((unsigned int)m.nodes.size() - 1);
    m.scenes.push_back(scene);
    tinygltf::TinyGLTF gltf;
    gltf.WriteGltfSceneToFile(&m, (main_fname + ".gltf").c_str(),
                               true,    // embedImages
                               true,    // embedBuffers
                               true,    // pretty print
                               false);  // write binary
}

int main()
{
    if constexpr (endian::native == endian::little)
        cout << "little-endian";
    else if constexpr (endian::native == endian::big)
        cout << "big-endian";
    else
        cout << "mixed-endian";
    printf("\n%d\n", pair<int, int>(1, 1) < pair<int, int>(1, 2));
    vector<string> filenames
    = 
    {
        "archer"
    };
    //string path = "./oss/";
    //string oss_ext = ".oss";
    //for (const auto& file : fs::directory_iterator(path)) {
    //    if (file.path().extension() == oss_ext) {
    //        cout << file.path().stem() << endl;
    //        filenames.push_back(file.path().stem().string());
    //    }
    //}
    unsigned int pos = 0;
    for (auto iter = filenames.begin(); iter != filenames.end(); iter++) {
        try {
            genOSSwithAttach("oss\\" + *iter + ".oss");
            //OssParser oss = OssParser("oss\\" + *iter + ".oss");
            //// oss.poseIt(pos);
            //oss.toGLTF(*iter + to_string(pos) + ".gltf", "oss\\" + *iter);
        } catch (...) {
            cout << "Error!" << endl;
        }
    }
    return 0;
}


