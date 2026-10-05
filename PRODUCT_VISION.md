# PRODUCT VISION
IMPORTANT: It is absolutely forbidden for AI to modify this document.

We are working on levitating chess idea - chess that use magnetic levitation (like a planar motor) to levitate over the board.
You can reference to pdf files in the project for more info about planar motors design.

The pieces should have halback permanent magnet arrays in them.
The board should have coils under it to levitate the pieces.
On the sides of the board there should be a buffer space for eaten pieces. It should fit in 32 pieces.

BOM cost should be calculated at a production volume of 100 boards.

Please also read PRODUCT_TESTING.md - it contains testing patterns that we require.

Constraints:

C1: The chess do not levitate constantly - in the normal game a figure only levitates when it moves: it lifts off it flies to the target place, it lands.

C2: The most power consuming action required is when all chess figures must go to their places at once, from whereever they are on the board or on the buffer. In this scenario pieces are allowed not to levitate, but slide instead.

C3: 6 DoF must be supported for all pieces during levitation.

C4: To create the "magical feeling" everything should happen in silence. If we must have fans they should be very very silent.

C5: Figures must not snap together (which is a risk because they have permanent magnets in them). We want to make sure that if we put two figures on the powered-off board in the way that bases of these figures touch, then if we move one figure, another figure do not move.

C7: Every reachable spot of the surface is always safe to touch — a hand may lift any resting piece at any moment, so no cell may ever exceed the brief-touch limit, covered or not.

C8: The side of the board with the buffer for eaten pieces is also "motorized" by the planar motor. When the game is over all pieces can go back automatically from the buffer to the field.
