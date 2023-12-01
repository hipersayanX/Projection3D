/* 3D to 2D projection, example.
 * Copyright (C) 2023  Gonzalo Exequiel Pedone
 *
 * Webcamoid is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Webcamoid is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Webcamoid. If not, see <http://www.gnu.org/licenses/>.
 */

#include <QKeyEvent>
#include <QPainter>
#include <QTimer>
#include <iostream>

#include "mainwindow.h"
#include "./ui_mainwindow.h"

using VectorCoordsType = qreal;

// Calculate the diferential vector between two vectors with 'vect2' as
// starting point.
template<typename T>
inline QVector<T> operator -(const QVector<T> &vect1, const QVector<T> &vect2)
{
    QVector<T> sum;
    auto size = qMin(vect1.size(), vect2.size());

    for (int i = 0; i < size; ++i)
        sum << vect1[i] - vect2[i];

    return sum;
}

// Scale vector by a 'k' factor.
template<typename K, typename T>
inline QVector<T> operator *(K k, const QVector<T> &vect)
{
    QVector<T> sum;

    for (auto &v: vect)
        sum << k * v;

    return sum;
}

// Calculate the dot product between two vectors.
template<typename T>
inline T operator *(const QVector<T> &vect1, const QVector<T> &vect2)
{
    T dot = 0;
    auto size = qMin(vect1.size(), vect2.size());

    for (int i = 0; i < size; ++i)
        dot += vect1[i] * vect2[i];

    return dot;
}

// Scale vector by a '1/k' factor.
template<typename T, typename K>
inline QVector<T> operator /(const QVector<T> &vect, K k)
{
    QVector<T> sum;

    for (auto &v: vect)
        sum << v / k;

    return sum;
}

/* Helper console out functions */

template<typename T>
std::ostream &operator <<(std::ostream &os, const QVector<T> &vect)
{
    bool first = true;
    os << "(";

    for (auto &v: vect) {
        if (first)
            first = false;
        else
            os << ", ";

        os << v;
    }

    os << ")";

    return os;
}

std::ostream &operator <<(std::ostream &os, const QString &vect)
{
    os << vect.toStdString().c_str();

    return os;
}

class MainWindowPrivate
{
    public:
        QImage m_frame;
        QTimer m_timer;

        /* Viewer coordinates */

        // Viewer position
        QVector<qreal> m_viewerPosition {0.0, 0.0, 100.0};

        // Distance from the focus to the viewer
        VectorCoordsType m_focalDistance {50.0};

        // View angles relative to the Z axis of the viewer.
        QVector<VectorCoordsType> m_viewAngles {0.0 * M_PI / 180.0,
                                                0.0 * M_PI / 180.0};

        // The size of the view window.
        // The view window is a plane positioned in the XY plane relative to
        // the viewer, with center in the ending of the focal vector
        QVector<VectorCoordsType> m_viewWindow {800.0, 600.0};

        /* Animation time for the circle */

        qreal m_T {5};
        qreal m_t {0};

        // Calculate the focal vector
        // For calculating the focal vector start with a vector positioned in
        // the Z axis with a module equal to the focal distance, that is
        // (0, 0, distance). Then rotate the vector each time in the direction
        // of the axis, the rotation angles are relative to the relative Z axis
        // of the viewer.
        template<typename T, typename U>
        inline static QVector<T> focalVector(T distance,
                                             const QVector<U> &angles)
        {
            QVector<T> focal;
            U sumTan = 0;
            bool infSumTan = false;

            for (auto &angle: angles) {
                if (qFuzzyIsNull(qCos(angle))) {
                    infSumTan = true;

                    break;
                }

                auto tan = qTan(angle);
                sumTan += tan * tan;
            }

            auto z = infSumTan? 0: distance / qSqrt(sumTan + 1.0);

            for (auto &angle: angles)
                if (qFuzzyIsNull(qCos(angle)))
                    focal << distance;
                else
                    focal << z * qTan(angle);

            focal << z;

            return focal;
        }

        // Sum up two vectors
        template<typename T>
        inline static QVector<T> add(const QVector<T> &vect1,
                                     const QVector<T> &vect2)
        {
            QVector<T> sum;
            auto size = qMin(vect1.size(), vect2.size());

            for (int i = 0; i < size; ++i)
                sum << vect1[i] + vect2[i];

            return sum;
        }

        // Calculate the module of the vector
        template<typename T>
        inline static T module(const QVector<T> &vect)
        {
            return qSqrt(vect * vect);
        }

        // Project the point in the focal plane.
        // Find the intersection between the focal plane and the rect defined by
        // the viewer position and the point, then substract the intersection
        // point and the focal point.
        template<typename T>
        inline static QVector<T> projection(const QVector<T> &point,
                                            const QVector<T> &focalPoint)
        {
            auto dot = point * focalPoint;

            if (qAbs(dot) < 0.001) {
                if (dot < 0.0)
                    dot = -0.001;
                else
                    dot = 0.001;
            }

            auto dotVp = focalPoint * focalPoint;
            QVector<T> proj;
            auto size = qMin(point.size(), focalPoint.size());

            for (int i = 0; i < size; ++i)
                proj << (point[i] * (dotVp - focalPoint[i] * focalPoint[i])
                         - focalPoint[i] * (dot - point[i] * focalPoint[i]))
                        / dot;

            return proj;
        }

        // Rotate the vector so that the focal plane becomes parallel to the
        // global XY plane, so then the coordinates becomes relative to the
        // global XY plane.
        template<typename T>
        inline static QVector<T> normalize(const QVector<T> &point,
                                           const QVector<T> &viewPoint)
        {
            auto p = point;
            auto last = qMin(point.size(), viewPoint.size()) - 1;
            auto div = MainWindowPrivate::module(viewPoint);

            if (div < 0.001)
                div = 0.001;

            for (int i = 0; i < last; ++i) {
                if (qFuzzyIsNull(viewPoint[i]) && qFuzzyIsNull(viewPoint[last]))
                    continue;

                auto x = (p[i] * viewPoint[last] - p[last] * viewPoint[i]) / div;
                auto z = (p[i] * viewPoint[i] + p[last] * viewPoint[last]) / div;
                p[i] = x;
                p[last] = z;
            }

            return p;
        }

        // Set the focal point as the center of the window, then calculate the
        // coordinates relative to it.
        template<typename T>
        inline static QVector<T> windowCoords(const QVector<T> &normalized,
                                              const QVector<T> &windowSize)
        {
            return MainWindowPrivate::add(normalized, 0.5 * windowSize);
        }
};

MainWindow::MainWindow(QWidget *parent):
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    this->d = new MainWindowPrivate;
    ui->setupUi(this);
    this->d->m_frame = {qRound(this->d->m_viewWindow[0]),
                        qRound(this->d->m_viewWindow[1]),
                        QImage::Format_RGBA8888};
    this->d->m_frame.fill(0);

    connect(&this->d->m_timer,
            &QTimer::timeout,
            this,
            [this] () {
        //this->d->m_frame.fill(0);
        QPainter painter;
        painter.begin(&this->d->m_frame);

        // Csalculate the focal vector
        auto focalVector =
                MainWindowPrivate::focalVector(this->d->m_focalDistance,
                                               this->d->m_viewAngles);

        // Calculate the point at which the viewer is looking at.
        auto viewPoint =
                MainWindowPrivate::add(focalVector, this->d->m_viewerPosition);

        std::cout << "View Position:" << this->d->m_viewerPosition << std::endl;
        std::cout << "View Point:" << viewPoint << std::endl;
        std::cout << "Focal Vector:" << focalVector << std::endl;

        /* First Draw a cube */

        // Each side of the cube will be 100 px
        static const VectorCoordsType scale = 50.0;

        // Vectices of the cube
        static const QVector<QVector<VectorCoordsType>> vertices {
            { 1.0,  1.0,  1.0},
            { 1.0, -1.0,  1.0},
            {-1.0, -1.0,  1.0},
            {-1.0,  1.0,  1.0},
            { 1.0,  1.0, -1.0},
            { 1.0, -1.0, -1.0},
            {-1.0, -1.0, -1.0},
            {-1.0,  1.0, -1.0},
        };

        // Edges of the cube
        static const QVector<QPair<int, int>> edges {
            {0, 1},
            {0, 3},
            {0, 4},
            {2, 1},
            {2, 3},
            {2, 6},
            {5, 1},
            {5, 4},
            {5, 6},
            {7, 3},
            {7, 4},
            {7, 6},
        };

        // Convert the 3D point to 2D coordinates
        QVector<QVector<VectorCoordsType>> paintedVertices;
        int i = 0;

        for (auto &vertex: vertices) {
            // Do the calculations
            auto scaledVertice = scale * vertex;
            auto projected =
                    MainWindowPrivate::projection(scaledVertice - this->d->m_viewerPosition,
                                                  focalVector);
            auto normalized =
                    MainWindowPrivate::normalize(projected, focalVector);
            auto painted =
                    MainWindowPrivate::windowCoords(normalized,
                                                    this->d->m_viewWindow);
            paintedVertices << painted;

            // Distance from the vertex to the viewer
            auto distance =
                    MainWindowPrivate::module(scaledVertice
                                              - this->d->m_viewerPosition);

            std::cout << "Cube Vertex " << i << ":" << std::endl;
            std::cout << "    Vertex Position:" << scaledVertice << std::endl;
            std::cout << "    Projected:" << projected << std::endl;
            std::cout << "    Normalized:" << normalized << std::endl;
            std::cout << "    Painted:" << painted  << std::endl;
            std::cout << "    Distance:" << distance << std::endl;

            i++;
        }

        // Draw the cube edges
        for (auto &edge: edges) {
            painter.setPen(QColor(255, 0, 0));
            auto &v1 = paintedVertices[edge.first];
            auto &v2 = paintedVertices[edge.second];
            painter.drawLine(v1[0], v1[1], v2[0], v2[1]);
        }

        /* Draw a circle parallel to the XZ plane */

        // Calculate the circle particle position each time
        static const VectorCoordsType radius = 50.0;
        static const QVector<VectorCoordsType> particleCenter = {0.0, 0.0, 0.0};

        auto a = 2.0 * M_PI * this->d->m_t / this->d->m_T;

        // Do the calculations
        auto particlePos =
                MainWindowPrivate::add({radius * qSin(a),
                                        0.0,
                                        radius * qCos(a)},
                                       particleCenter);
        auto projected =
                MainWindowPrivate::projection(particlePos
                                              - this->d->m_viewerPosition,
                                              focalVector);
        auto normalized =
                MainWindowPrivate::normalize(projected, focalVector);
        auto painted = MainWindowPrivate::windowCoords(normalized,
                                                       this->d->m_viewWindow);
        auto distance =
                MainWindowPrivate::module(particlePos
                                          - this->d->m_viewerPosition);

        // Paint the particle
        painter.setBrush(QColor(255, 0, 0));
        painter.drawRect(painted[0], painted[1], 4, 4);

        std::cout << "Circle:" << std::endl;
        std::cout << "    Particle Position:" << particlePos << std::endl;
        std::cout << "    Projected:" << projected << std::endl;
        std::cout << "    Normalized:" << normalized << std::endl;
        std::cout << "    Painted:" << painted  << std::endl;
        std::cout << "    Distance:" << distance << std::endl;

        std::cout << std::endl;

        painter.end();

        // Update the scene
        ui->label->setPixmap(QPixmap::fromImage(this->d->m_frame));

        this->d->m_t += this->d->m_T / 100;
    });
    this->d->m_timer.setInterval(33);
    this->d->m_timer.start();
}

MainWindow::~MainWindow()
{
    delete this->d;
    delete ui;
}

bool MainWindow::event(QEvent *event)
{
    if (event->type() == QEvent::KeyPress) {
        this->d->m_frame.fill(0);
        auto keyEvent = dynamic_cast<QKeyEvent *>(event);

        // Distance increment
        static const VectorCoordsType k = 5.0;

        // Angle increment
        static const auto a = 1.0 * k * M_PI / 180.0;

        auto focalVector =
                MainWindowPrivate::focalVector(this->d->m_focalDistance,
                                               this->d->m_viewAngles);
        auto zp = focalVector / MainWindowPrivate::module(focalVector);

        switch (keyEvent->key()) {
        case Qt::Key_W: // Move Forward
            this->d->m_viewerPosition =
                    MainWindowPrivate::add(this->d->m_viewerPosition, -k * zp);
            break;
        case Qt::Key_A: // Move Left
            this->d->m_viewerPosition =
                MainWindowPrivate::add(this->d->m_viewerPosition, {-k, 0.0, 0.0});
            break;
        case Qt::Key_S: // Move Backward
            this->d->m_viewerPosition =
                    MainWindowPrivate::add(this->d->m_viewerPosition, k * zp);
            break;
        case Qt::Key_D: // Move Right
            this->d->m_viewerPosition =
                MainWindowPrivate::add(this->d->m_viewerPosition, {k, 0.0, 0.0});
            break;
        case Qt::Key_Q: // Move Up
            this->d->m_viewerPosition =
                MainWindowPrivate::add(this->d->m_viewerPosition, {0.0, k, 0.0});
            break;
        case Qt::Key_E: // Move Down
            this->d->m_viewerPosition =
                MainWindowPrivate::add(this->d->m_viewerPosition, {0.0, -k, 0.0});
            break;

        case Qt::Key_I: // Tilt Up
            this->d->m_viewAngles =
                MainWindowPrivate::add(this->d->m_viewAngles, {0.0, -a});
            break;
        case Qt::Key_J: // Tilt Left
            this->d->m_viewAngles =
                MainWindowPrivate::add(this->d->m_viewAngles, {-a, 0.0});
            break;
        case Qt::Key_K: // Tilt Down
            this->d->m_viewAngles =
                MainWindowPrivate::add(this->d->m_viewAngles, {0.0, a});
            break;
        case Qt::Key_L: // Tilt Right
            this->d->m_viewAngles =
                MainWindowPrivate::add(this->d->m_viewAngles, {a, 0.0});
            break;
        case Qt::Key_U: // Zoom In
            this->d->m_focalDistance += k;
            break;
        case Qt::Key_O: // Zoom Down
            this->d->m_focalDistance -= k;
            break;
        default:
            return QMainWindow::event(event);
        }

        event->accept();

        return true;
    }

    return QMainWindow::event(event);
}

void MainWindow::on_resetView_clicked()
{
    this->d->m_viewerPosition = {0.0, 0.0, 100.0};
    this->d->m_focalDistance = 50.0;
    this->d->m_viewAngles = {0.0 * M_PI / 180.0, 0.0 * M_PI / 180.0};
    this->d->m_viewWindow = {800.0, 600.0};
    this->d->m_frame.fill(0);
}
