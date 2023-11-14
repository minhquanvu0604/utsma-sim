[User Manual](https://doc.cgal.org/latest/Triangulation_2/index.html#Section_2D_Triangulations_Flexibility)

2.1 The Set of Faces

The single unbounded face of this partition is the complementary of the convex hull. [...]
Therefore, a fictitious vertex, called the **infinite vertex** is added to the triangulation as well as **infinite edges\*** and infinite faces incident to it.

2.2 A Representation Based on Faces and Vertices

3 Software Design

4 Basic Triangulations

4.1 Description
The class Triangulation_2<Traits,Tds> serves as a base class for the other 2D triangulations classes and implements the user interface to a triangulation.

The vertices and faces of the triangulations are accessed through handles, iterators and circulators. A handle is a model of the concept Handle which basically offers the two dereference operators * and ->. A circulator is a type devoted to visit circular sequences. Handles are used whenever the accessed element is not part of a sequence. Iterators and circulators are used to visit all or parts of the triangulation.

The iterators and circulators are all bidirectional and non mutable. The circulators and iterators are convertible to the handles with the same value type, so that when calling a member function, any handle type argument can be replaced by an iterator or a circulator with the same value type.