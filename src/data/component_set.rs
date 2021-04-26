use crate::data::Index;

// TODO ECS: use this to handle optional components properly.
// pub trait OptionalComponentSet<T> {
//     fn get(&self, handle: Index) -> Option<&T>;
// }

pub trait ComponentSetOption<T> {
    fn get(&self, handle: Index) -> Option<&T>;
}

pub trait ComponentSet<T>: ComponentSetOption<T> {
    fn size_hint(&self) -> usize;
    // TODO ECS: remove this, its only needed by the query pipeline update
    //           which should only take the modified colliders into account.
    fn for_each(&self, f: impl FnMut(Index, &T));
    fn index(&self, handle: Index) -> &T {
        self.get(handle).unwrap()
    }
}

pub trait ComponentSetMut<T>: ComponentSet<T> {
    fn map_mut_internal<Result>(
        &mut self,
        handle: crate::data::Index,
        f: impl FnOnce(&mut T) -> Result,
    ) -> Option<Result>;
    fn set_internal(&mut self, handle: crate::data::Index, val: T);
}

pub trait BundleSet<'a, T> {
    fn index_bundle(&'a self, handle: Index) -> T;
}

impl<'a, T, A, B> BundleSet<'a, (&'a A, &'a B)> for T
where
    T: ComponentSet<A> + ComponentSet<B>,
{
    #[inline(always)]
    fn index_bundle(&'a self, handle: Index) -> (&'a A, &'a B) {
        (self.index(handle), self.index(handle))
    }
}

impl<'a, T, A, B, C> BundleSet<'a, (&'a A, &'a B, &'a C)> for T
where
    T: ComponentSet<A> + ComponentSet<B> + ComponentSet<C>,
{
    #[inline(always)]
    fn index_bundle(&'a self, handle: Index) -> (&'a A, &'a B, &'a C) {
        (self.index(handle), self.index(handle), self.index(handle))
    }
}

impl<'a, T, A, B, C, D> BundleSet<'a, (&'a A, &'a B, &'a C, &'a D)> for T
where
    T: ComponentSet<A> + ComponentSet<B> + ComponentSet<C> + ComponentSet<D>,
{
    #[inline(always)]
    fn index_bundle(&'a self, handle: Index) -> (&'a A, &'a B, &'a C, &'a D) {
        (
            self.index(handle),
            self.index(handle),
            self.index(handle),
            self.index(handle),
        )
    }
}

impl<'a, T, A, B, C, D, E> BundleSet<'a, (&'a A, &'a B, &'a C, &'a D, &'a E)> for T
where
    T: ComponentSet<A> + ComponentSet<B> + ComponentSet<C> + ComponentSet<D> + ComponentSet<E>,
{
    #[inline(always)]
    fn index_bundle(&'a self, handle: Index) -> (&'a A, &'a B, &'a C, &'a D, &'a E) {
        (
            self.index(handle),
            self.index(handle),
            self.index(handle),
            self.index(handle),
            self.index(handle),
        )
    }
}

impl<'a, T, A, B, C, D, E, F> BundleSet<'a, (&'a A, &'a B, &'a C, &'a D, &'a E, &'a F)> for T
where
    T: ComponentSet<A>
        + ComponentSet<B>
        + ComponentSet<C>
        + ComponentSet<D>
        + ComponentSet<E>
        + ComponentSet<F>,
{
    #[inline(always)]
    fn index_bundle(&'a self, handle: Index) -> (&'a A, &'a B, &'a C, &'a D, &'a E, &'a F) {
        (
            self.index(handle),
            self.index(handle),
            self.index(handle),
            self.index(handle),
            self.index(handle),
            self.index(handle),
        )
    }
}
