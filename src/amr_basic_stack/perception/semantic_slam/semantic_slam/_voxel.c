/* Exact first-occurrence reduction for contiguous native uint64 voxel keys. */
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <stdint.h>
#include <string.h>

static PyObject *first_indices(PyObject *self, PyObject *arg)
{
    (void)self;
    Py_buffer view;
    if (PyObject_GetBuffer(arg, &view, PyBUF_FORMAT | PyBUF_C_CONTIGUOUS) < 0)
        return NULL;
    if (view.ndim != 1 || view.itemsize != 8 ||
        (strcmp(view.format, "L") != 0 && strcmp(view.format, "Q") != 0)) {
        PyBuffer_Release(&view);
        PyErr_SetString(PyExc_ValueError, "Expected contiguous native uint64 keys");
        return NULL;
    }
    size_t count = (size_t)view.len / 8;
    if (!count) {
        PyBuffer_Release(&view);
        return PyBytes_FromStringAndSize("", 0);
    }
    if (count > (size_t)PY_SSIZE_T_MAX / 32) {
        PyBuffer_Release(&view);
        return PyErr_NoMemory();
    }
    size_t capacity = 1;
    while (capacity < count * 2) capacity <<= 1;
    uint64_t *table = PyMem_Malloc(capacity * sizeof(uint64_t));
    unsigned char *used = PyMem_Calloc(capacity, 1);
    int64_t *indices = PyMem_Malloc(count * sizeof(int64_t));
    if (!table || !used || !indices) {
        PyMem_Free(table); PyMem_Free(used); PyMem_Free(indices);
        PyBuffer_Release(&view);
        return PyErr_NoMemory();
    }
    size_t selected = 0;
    const unsigned char *keys = view.buf;
    for (size_t i = 0; i < count; ++i) {
        uint64_t key;
        memcpy(&key, keys + i * 8, 8); /* Buffers need not be aligned. */
        uint64_t hash = key;
        hash ^= hash >> 30; hash *= UINT64_C(0xbf58476d1ce4e5b9);
        hash ^= hash >> 27; hash *= UINT64_C(0x94d049bb133111eb);
        hash ^= hash >> 31;
        size_t slot = (size_t)hash & (capacity - 1);
        while (used[slot] && table[slot] != key)
            slot = (slot + 1) & (capacity - 1);
        if (!used[slot]) {
            used[slot] = 1;
            table[slot] = key;
            indices[selected++] = (int64_t)i;
        }
    }
    PyObject *result = PyBytes_FromStringAndSize((const char *)indices, selected * 8);
    PyMem_Free(table); PyMem_Free(used); PyMem_Free(indices);
    PyBuffer_Release(&view);
    return result;
}

static PyMethodDef methods[] = {
    {"first_indices", first_indices, METH_O, "Return first-occurrence indices as native int64 bytes."},
    {NULL, NULL, 0, NULL}
};
static struct PyModuleDef module = {
    PyModuleDef_HEAD_INIT, "_voxel", "Exact voxel-key reduction.", -1, methods
};
PyMODINIT_FUNC PyInit__voxel(void) { return PyModule_Create(&module); }
