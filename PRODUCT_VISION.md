We are working on levitating chess idea - chess that use magnetic levitation (like a planar motor) to levitate over the board.
You can reference to pdf files in the project for more info about planar motors design.

The pieces should have halback permanent magnet arrays in them.
The board should have coils under it to levitate the pieces.
On the sides of the board there should be a buffer space for eaten pieces. It should fit in 32 pieces.

Constraints:
C1: The chess do not levitate constantly - in the normal game a figure only levitates when it moves: it lifts off it flies to the target place, it lands.

C2: The most power consuming action required is when all chess figures must go to their places at once, from whereever they are on the board. For this use-case at least 5 Degrees Of Freedom is enough for every piece.

C3: When single piece is controlled, 6 DoF should be supported.

C4: There should be no fans - to create the "magical feeling" everything should happen in silence.

C5: Figures must not snap together. We want to make sure that if we put two figures on the powered-off board in the way that bases of these figures touch, then if we move one figure, another figure do not move.
