#pragma once
#include "glad/glad.h"


namespace utils {

    template<typename T, unsigned int N>
    struct Attribute {
        typedef T type;
        static constexpr unsigned int count = N;
        static constexpr unsigned int size = N * sizeof(T);
    };

    template<typename... Args>
    struct VertexArray {};

    template<typename T, unsigned int N, typename... Args>
    struct VertexArray<Attribute<T, N>, Args...> {
    public:
        template<typename U>
        VertexArray(U* data, unsigned int size) {
            glGenVertexArrays(1, &vertexArrayId);
            glBindVertexArray(vertexArrayId);
            glGenBuffers(1, &bufferId);
            glBindBuffer(GL_ARRAY_BUFFER, bufferId);
            glBufferData(GL_ARRAY_BUFFER, size * vertexSize, (const void*) data, GL_DYNAMIC_DRAW);

            Init<History<0, 0>, Attribute<T, N>, Args...>()();
        }

        template<typename... Args>
        struct Init {
        public:
            Init() {}
            void operator()() {
                throw("Not implemented");
            }
        };

        template<unsigned int I, unsigned int Offset>
        struct History {
            static constexpr unsigned int index = I;
            static constexpr unsigned int offset = Offset;
        };

        template<unsigned int Index, unsigned int Offset, typename Type, unsigned int Count, typename... Args>
        struct Init<History<Index, Offset>, Attribute<Type, Count>, Args...> {
            inline void operator()() {
                std::cout << typeid(Type).name() << std::endl;
                glVertexAttribPointer(Index, Count, GetGLType<Type>(), GL_FALSE, vertexSize, (const void*)Offset);
                glEnableVertexAttribArray(Index);
                std::cout << Index << "  " << typeid(Type).name() << "  " << Count << "  " << vertexSize << "  " << Offset << std::endl;
                constexpr unsigned int Nextindex = Index + 1;
                constexpr unsigned int NextOffset = Offset + Attribute<T, N>::size;
                Init<History<Nextindex, NextOffset>, Args...>()();
            }
        };

        template<unsigned int Index, unsigned int Offset>
        struct Init<History<Index, Offset>> {
            inline void operator()() {}
        };

        inline unsigned int getVertexSize() {
            return Size<Attribute<T, N>, Args...>::size;
        }

        template<typename... Args>
        struct Size {
        };

        template<typename T, unsigned int N, typename... Args>
        struct Size<Attribute<T, N>, Args...> {
            static constexpr unsigned int size = Attribute<T, N>::size + Size<Args...>::size;
        };

        template<typename T, unsigned int N>
        struct Size<Attribute<T, N>> {
            static constexpr unsigned int size = Attribute<T, N>::size;
        };

        template<typename T>
        static unsigned int GetGLType() {
            throw("Not implemented");
        };

        template<>
        static unsigned int GetGLType<float>() {
            return GL_FLOAT;
        };

        template<>
        static unsigned int GetGLType<unsigned int>() {
            return GL_UNSIGNED_INT;
        };

        template<>
        static unsigned int GetGLType<unsigned char>() {
            return GL_UNSIGNED_BYTE;
        };

        static constexpr unsigned int vertexSize = Size<Attribute<T, N>, Args...>::size;

    public:
        unsigned int vertexArrayId;
        unsigned int bufferId;
    };
}
