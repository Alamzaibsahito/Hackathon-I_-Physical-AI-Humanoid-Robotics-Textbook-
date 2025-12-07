import React from 'react';
import DocItem from '@theme-original/DocItem';
import DifficultyToggle from '@site/src/components/DifficultyToggle';
import TranslateButton from '@site/src/components/TranslateButton';

export default function DocItemWrapper(props) {
  return (
    <>
      <DifficultyToggle />
      <TranslateButton />
      <DocItem {...props} />
    </>
  );
}
