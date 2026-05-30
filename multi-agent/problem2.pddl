(define (problem p2-network)
    (:domain MultiRobotRunner)

    (:objects
        robot1 robot2 - robot

        g00 g01 g02
        g10 g11 g12
        g20 g21 g22 - gridsquare

        north east south west - direction
    )

    (:init
        ; Locations of Robots
        (robot-at robot1 g10)
        (robot-at robot2 g01)

        ; Robot rotations
        (robot-facing robot1 east)
        (robot-facing robot2 south)

        ; Goal locations
        (goal-location robot1 g12)
        (goal-location robot2 g21)

        ; Turn order
        (first-robot robot1)
        (last-robot robot2)
        (next-turn robot1 robot2)

        ; Initial phase / turn
        (phase-select)
        (turn robot1)

        ; east
        (gridsquare-connected g00 g01 east)
        (gridsquare-connected g01 g02 east)
        (gridsquare-connected g10 g11 east)
        (gridsquare-connected g11 g12 east)
        (gridsquare-connected g20 g21 east)
        (gridsquare-connected g21 g22 east)

        ; west
        (gridsquare-connected g01 g00 west)
        (gridsquare-connected g02 g01 west)
        (gridsquare-connected g11 g10 west)
        (gridsquare-connected g12 g11 west)
        (gridsquare-connected g21 g20 west)
        (gridsquare-connected g22 g21 west)

        ; south
        (gridsquare-connected g00 g10 south)
        (gridsquare-connected g10 g20 south)
        (gridsquare-connected g01 g11 south)
        (gridsquare-connected g11 g21 south)
        (gridsquare-connected g02 g12 south)
        (gridsquare-connected g12 g22 south)

        ; north
        (gridsquare-connected g10 g00 north)
        (gridsquare-connected g20 g10 north)
        (gridsquare-connected g11 g01 north)
        (gridsquare-connected g21 g11 north)
        (gridsquare-connected g12 g02 north)
        (gridsquare-connected g22 g12 north)

        ; rotations
        (rotation-adjacent north east)
        (rotation-adjacent east south)
        (rotation-adjacent south west)
        (rotation-adjacent west north)

        (rotation-adjacent east north)
        (rotation-adjacent south east)
        (rotation-adjacent west south)
        (rotation-adjacent north west)

        (= (total-cost) 0)
    )

    (:goal
        (and
            (robot-at robot1 g12)
            (robot-at robot2 g21)
        )
    )

    (:metric minimize (total-cost))
)
