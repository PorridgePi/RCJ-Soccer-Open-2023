import turtle
from random import randint


robot = turtle.Turtle()
ball = turtle.Turtle()

ball.goto(0, 0)
# make ball look like ball
ball.shape("circle")
# smaller sized
ball.shapesize(0.5, 0.5)

turtle.screensize(canvwidth=158, canvheight=219,
                  bg="green")

ballX, ballY = 0, 0
robot.speed(0)

robot.speed("fastest")
ball.speed("fastest")


while True:
    oriX, oriY = randint(-158, 158), randint(-219, 219)
    robot.penup()
    robot.goto(oriX, oriY)
    robot.pendown()

    while robot.distance(ballX, ballY) > 3:
        ballX = ball.xcor()
        ballY = ball.ycor()
        ballDistance = robot.distance(ballX, ballY)
        ballAngle = robot.towards(ballX, ballY)
        targetAngle = ballAngle * (2 - pow(ballDistance, 0.8))

        if (ballAngle >= 0 and ballAngle < 90):
            targetAngle = ballAngle * (2 - pow(ballDistance / 350, 0.8))
        elif (ballAngle >= 90 and ballAngle < 180):
            targetAngle = ballAngle + 90 * (1 - pow(ballDistance / 350, 0.6))
        elif (ballAngle >= 180 and ballAngle < 270):
            targetAngle = ballAngle - 90 * (1 - pow(ballDistance / 350, 0.6))
        elif (ballAngle >= 270 and ballAngle < 360):
            targetAngle = 360 - (360 - ballAngle) * (2 - pow(ballDistance / 350, 0.8))


        robot.setheading(targetAngle)
        # print(ballAngle)
        robot.forward(5)

        ball.forward(0.5)
        # ball.setheading(randint(, 360))
        if ball.xcor() > 158 or ball.xcor() < -158 or ball.ycor() > 219 or ball.ycor() < -219:
            ball.setheading(randint(0, 360))
            ball.forward(10)
