;It is recommended that you install the misc-pddl-generators plugin 
;and then use the Network generator to create the graph
(define (problem p3-network)
    (:domain MultiRobotRunner)

    (:objects
        robot1 robot2 - robot

        g00 g01 g02
        g10 g11 g12 - gridsquare

        north east south west - direction
    )

    (:init
        ; Locations of Robots
        (robot-at robot1 g00)
        (robot-at robot2 g02)

        ; Robot rotations
        (robot-facing robot1 east)
        (robot-facing robot2 west)

        ; Goal locations
        (goal-location robot1 g02)
        (goal-location robot2 g00)

        ; Turn order
        (first-robot robot1)
        (last-robot robot2)
        (next-turn robot1 robot2)

        ; Initial phase / turn
        (phase-select)
        (turn robot1)

        ; Gridsquare connections
        ; top row
        (gridsquare-connected g00 g01 east)
        (gridsquare-connected g01 g02 east)

        (gridsquare-connected g01 g00 west)
        (gridsquare-connected g02 g01 west)

        ; bottom row
        (gridsquare-connected g10 g11 east)
        (gridsquare-connected g11 g12 east)

        (gridsquare-connected g11 g10 west)
        (gridsquare-connected g12 g11 west)

        ; vertical connections
        (gridsquare-connected g00 g10 south)
        (gridsquare-connected g01 g11 south)
        (gridsquare-connected g02 g12 south)

        (gridsquare-connected g10 g00 north)
        (gridsquare-connected g11 g01 north)
        (gridsquare-connected g12 g02 north)

        ; Directions
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
            (robot-at robot1 g02)
            (robot-at robot2 g00)
        )
    )

    (:metric minimize (total-cost))
)
