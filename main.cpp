#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

#include "PhysicsEngine.h"
#include "snippetrender/SnippetRender.h"
#include "snippetrender/SnippetCamera.h"

using namespace physx;

#define SAFE_RELEASE(obj) \
{                         \
    if(obj)               \
    {                     \
        obj->release();   \
        obj = nullptr;    \
    }                     \
}

//////////////////////////////////////////////////////////////////
// Константы игры
//////////////////////////////////////////////////////////////////

namespace Game
{
    constexpr float TABLE_LENGTH = 2.4f;
    constexpr float TABLE_WIDTH = 1.2f;

    constexpr float TABLE_THICKNESS = 0.05f;

    constexpr float BORDER_HEIGHT = 0.10f;
    constexpr float BORDER_THICKNESS = 0.08f;

    constexpr float BALL_RADIUS = 0.03f;
    constexpr float BALL_DENSITY = 400.0f;

    constexpr float SHOT_FORCE = 0.2f;

    constexpr float AIM_STEP = 0.05f;

    constexpr float POCKET_RADIUS = 0.1f;

    constexpr float STOP_SPEED = 0.05f;
}

//////////////////////////////////////////////////////////////////
// Глобальные переменные
//////////////////////////////////////////////////////////////////

PhysicsEngine* engine = nullptr;
PhysicsEngine* physicsEngine = nullptr;

Snippets::Camera* camera = nullptr;

PxRigidDynamic* cueBall = nullptr;

std::vector<PxRigidDynamic*> balls;

float aimAngle = 0.0f;

bool gameOver = false;

//////////////////////////////////////////////////////////////////
// Возвращает направление удара
//////////////////////////////////////////////////////////////////

PxVec3 GetShotDirection()
{
    return PxVec3(
        cosf(aimAngle),
        0.0f,
        sinf(aimAngle)
    ).getNormalized();
}

//////////////////////////////////////////////////////////////////
// Проверка движется ли шар
//////////////////////////////////////////////////////////////////

bool IsMoving(PxRigidDynamic* ball)
{
    if (!ball)
        return false;

    return ball->getLinearVelocity().magnitude()
        > Game::STOP_SPEED;
}

//////////////////////////////////////////////////////////////////
// Проверка движется ли хоть один шар
//////////////////////////////////////////////////////////////////

bool AnyBallMoving()
{
    if (IsMoving(cueBall))
        return true;

    for (auto ball : balls)
    {
        if (IsMoving(ball))
            return true;
    }

    return false;
}

//////////////////////////////////////////////////////////////////
// Создание одного шара
//////////////////////////////////////////////////////////////////

PxRigidDynamic* CreateBall(
    PxVec3 position,
    PxMaterial* material
)
{
    PxShape* shape =
        engine->CreateSphereShape(
            Game::BALL_RADIUS,
            material,
            eDYNAMIC
        );

    PxRigidDynamic* ball =
        engine->AddDynamicActor(
            shape,
            position,
            PxQuat(PxIdentity),
            Game::BALL_DENSITY
        );

    ball->setLinearDamping(0.3f);
    ball->setAngularDamping(0.3f);

    SAFE_RELEASE(shape);

    return ball;
}

//////////////////////////////////////////////////////////////////
// Создание стола
//////////////////////////////////////////////////////////////////

void CreateTable()
{
    PxMaterial* material =
        engine->GetMaterial(
            0.8f,
            0.7f,
            0.2f
        );

    // Основание стола

    PxShape* floorShape =
        engine->CreateBoxShape(
            PxVec3(
                Game::TABLE_LENGTH,
                Game::TABLE_THICKNESS,
                Game::TABLE_WIDTH
            ),
            material,
            eOBSTACLE
        );

    engine->AddStaticActor(
        floorShape,
        PxVec3(
            0,
            -Game::TABLE_THICKNESS / 2,
            0
        ),
        PxQuat(PxIdentity)
    );

    SAFE_RELEASE(floorShape);

    //////////////////////////////////////////////////////////
    // Верхний борт
    //////////////////////////////////////////////////////////

    PxShape* borderShape =
        engine->CreateBoxShape(
            PxVec3(
                Game::TABLE_LENGTH,
                Game::BORDER_HEIGHT,
                Game::BORDER_THICKNESS
            ),
            material,
            eOBSTACLE
        );

    engine->AddStaticActor(
        borderShape,
        PxVec3(
            0,
            Game::BORDER_HEIGHT / 2,
            Game::TABLE_WIDTH / 2 +
            Game::BORDER_THICKNESS / 2
        ),
        PxQuat(PxIdentity)
    );

    engine->AddStaticActor(
        borderShape,
        PxVec3(
            0,
            Game::BORDER_HEIGHT / 2,
            -Game::TABLE_WIDTH / 2 -
            Game::BORDER_THICKNESS / 2
        ),
        PxQuat(PxIdentity)
    );

    SAFE_RELEASE(borderShape);

    //////////////////////////////////////////////////////////
    // Левый и правый борта
    //////////////////////////////////////////////////////////

    borderShape =
        engine->CreateBoxShape(
            PxVec3(
                Game::BORDER_THICKNESS,
                Game::BORDER_HEIGHT,
                Game::TABLE_WIDTH
            ),
            material,
            eOBSTACLE
        );

    engine->AddStaticActor(
        borderShape,
        PxVec3(
            Game::TABLE_LENGTH / 2 +
            Game::BORDER_THICKNESS / 2,
            Game::BORDER_HEIGHT / 2,
            0
        ),
        PxQuat(PxIdentity)
    );

    engine->AddStaticActor(
        borderShape,
        PxVec3(
            -Game::TABLE_LENGTH / 2 -
            Game::BORDER_THICKNESS / 2,
            Game::BORDER_HEIGHT / 2,
            0
        ),
        PxQuat(PxIdentity)
    );

    SAFE_RELEASE(borderShape);

//////////////////////////////////////////////////////////
// Визуализация луз
//////////////////////////////////////////////////////////

    PxMaterial* pocketMaterial =
        engine->GetMaterial(
            0.0f,
            0.0f,
            0.0f
        );

    PxShape* pocketShape =
        engine->CreateSphereShape(
            Game::POCKET_RADIUS * 0.8f,
            pocketMaterial,
            eOBSTACLE,
            false,
            PxShapeFlag::eVISUALIZATION
        );

    float halfL = Game::TABLE_LENGTH / 2;
    float halfW = Game::TABLE_WIDTH / 2;

    std::vector<PxVec3> visualPockets =
    {
        PxVec3(-halfL, 0.01f, -halfW),
        PxVec3(halfL, 0.01f, -halfW),

        PxVec3(-halfL, 0.01f,  halfW),
        PxVec3(halfL, 0.01f,  halfW),

        PxVec3(0, 0.01f, -halfL / 2),
        PxVec3(0, 0.01f, halfL / 2)
    };

    for (const auto& p : visualPockets)
    {
        engine->AddStaticActor(
            pocketShape,
            p,
            PxQuat(PxIdentity)
        );
    }

    SAFE_RELEASE(pocketShape);
}

//////////////////////////////////////////////////////////////////
// Создание шаров
//////////////////////////////////////////////////////////////////

void CreateBalls()
{
    PxMaterial* ballMaterial =
        engine->GetMaterial(
            0.3f,
            0.3f,
            0.8f
        );

    //////////////////////////////////////////////////////////
    // Пирамида из 15 шаров
    //////////////////////////////////////////////////////////

    float startX = 0.5f;

    float step =
        Game::BALL_RADIUS * 2.3f;

    for (int row = 0; row < 5; row++)
    {
        for (int col = 0; col <= row; col++)
        {
            float z =
                (col - row * 0.5f) * step;

            float x =
                startX +
                row * step * 0.87f;

            PxRigidDynamic* ball =
                CreateBall(
                    PxVec3(
                        x,
                        Game::BALL_RADIUS,
                        z
                    ),
                    ballMaterial
                );

            balls.push_back(ball);
        }
    }

    //////////////////////////////////////////////////////////
    // Биток
    //////////////////////////////////////////////////////////

    cueBall =
        CreateBall(
            PxVec3(
                -0.8f,
                0.08f,
                0.0f
            ),
            ballMaterial
        );
}

//////////////////////////////////////////////////////////////////
// Проверка попадания в зону забивания
//////////////////////////////////////////////////////////////////

bool IsPocketPosition(PxVec3 position)
{
    float halfL =
        Game::TABLE_LENGTH / 2;

    float halfW =
        Game::TABLE_WIDTH / 2;

    std::vector<PxVec3> pockets =
    {
        PxVec3(-halfL,0,-halfW),
        PxVec3(halfL,0,-halfW),

        PxVec3(-halfL,0, halfW),
        PxVec3(halfL,0, halfW),

        PxVec3(0, 0.01f, -halfL / 2),
        PxVec3(0, 0.01f, halfL / 2)
    };

    for (auto& p : pockets)
    {
        PxVec3 pos2d(
            position.x,
            0,
            position.z
        );

        PxVec3 pocket2d(
            p.x,
            0,
            p.z
        );

        if ((pos2d - pocket2d).magnitude()
            < Game::POCKET_RADIUS)
        {
            return true;
        }
    }

    return false;
}

//////////////////////////////////////////////////////////////////
// Проверка забитых шаров
//////////////////////////////////////////////////////////////////

void CheckPocketedBalls()
{
    //////////////////////////////////////////////////////////
    // Проверяем биток
    //////////////////////////////////////////////////////////

    if (cueBall)
    {
        if (IsPocketPosition(
            cueBall->getGlobalPose().p))
        {
            std::cout
                << "GAME OVER"
                << std::endl;

            engine->MarkActor(cueBall);

            cueBall = nullptr;

            gameOver = true;

            return;
        }
    }

    //////////////////////////////////////////////////////////
    // Проверяем остальные шары
    //////////////////////////////////////////////////////////

    for (auto& ball : balls)
    {
        if (!ball)
            continue;

        if (IsPocketPosition(
            ball->getGlobalPose().p))
        {
            engine->MarkActor(ball);

            ball = nullptr;
        }
    }

    balls.erase(
        std::remove(
            balls.begin(),
            balls.end(),
            nullptr
        ),
        balls.end()
    );
}

//////////////////////////////////////////////////////////////////
// Проверка победы
//////////////////////////////////////////////////////////////////

void CheckWin()
{
    if (gameOver)
        return;

    if (balls.empty())
    {
        std::cout
            << "YOU WIN!"
            << std::endl;

        gameOver = true;
    }
}

//////////////////////////////////////////////////////////////////
// Создание сцены
//////////////////////////////////////////////////////////////////

void BuildScene()
{
    CreateTable();
    CreateBalls();
}

//////////////////////////////////////////////////////////////////
// Управление
//////////////////////////////////////////////////////////////////

void keyPressedCallback(
    unsigned char key,
    const PxTransform&
)
{

    if (gameOver)
        return;

    switch (toupper(key))
    {
    case 'J':
        aimAngle -= Game::AIM_STEP;
        break;

    case 'L':
        aimAngle += Game::AIM_STEP;
        break;

    case ' ':

        if (!AnyBallMoving())
        {
            cueBall->addForce(
                GetShotDirection()
                * Game::SHOT_FORCE,
                PxForceMode::eIMPULSE
            );
        }

        break;
    }
}

//////////////////////////////////////////////////////////////////
// Отрисовка кадра
//////////////////////////////////////////////////////////////////

void renderCallback()
{
    engine->Simulate(
        1.0f / 60.0f
    );

    CheckPocketedBalls();
    CheckWin();

    Snippets::startRender(camera);

    if (cueBall)
    {
        PxVec3 start =
            cueBall->getGlobalPose().p;

        PxVec3 end =
            start +
            GetShotDirection() * 0.5f;

        Snippets::DrawLine(
            start,
            end,
            PxVec3(1.0f, 1.0f, 0.0f)
        );
    }

    auto actors =
        engine->GetActors();

    if (!actors.empty())
    {
        Snippets::renderActors(
            actors.data(),
            (PxU32)actors.size()
        );
    }

    Snippets::finishRender();
}

//////////////////////////////////////////////////////////////////
// Завершение программы
//////////////////////////////////////////////////////////////////

void exitCallback()
{
    delete camera;
    delete engine;
}

//////////////////////////////////////////////////////////////////
// Точка входа
//////////////////////////////////////////////////////////////////

int main()
{
    camera =
        new Snippets::Camera(
            PxVec3(
                -2.0f,
                1.2f,
                0.0f
            ),
            PxVec3(
                1.0f,
                -0.6f,
                0.0f
            )
        );

    Snippets::setupDefault(
        "Billiards",
        camera,
        keyPressedCallback,
        renderCallback,
        exitCallback
    );

    engine = new PhysicsEngine();
    physicsEngine = engine;

    BuildScene();

    std::cout
        << "Controls:\n"
        << "J - left\n"
        << "L - right\n"
        << "SPACE - shoot\n";

    glutMainLoop();

    return 0;
}